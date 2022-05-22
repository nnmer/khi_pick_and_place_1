using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Rs007Control;
using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
using Unity.Robotics.ROSTCPConnector;


public enum MmSegForm { None, Straight, Curved }


[Serializable]
public class MmPathSeg
{
    string name;
    public static bool inited = false;
    static float sphrad = 0.3f;
    static Dictionary<string, float> angleval;
    static Dictionary<(string, string), string> rot90;
    static Dictionary<string, Vector3> lenVek;
    static Dictionary<string, float> compassSigns;
    static List<string> compassPoints = new List<string>() { "s", "n", "e", "w" };
    static List<string> compassDirections = new List<string>() { "cw", "ccw" };



    public static void InitDicts()
    {
        Debug.Log("InitDicts");
        rot90 = new Dictionary<(string, string), string>();
        rot90[("cw", "s")] = "w";
        rot90[("cw", "w")] = "n";
        rot90[("cw", "n")] = "e";
        rot90[("cw", "e")] = "s";
        rot90[("ccw", "s")] = "e";
        rot90[("ccw", "e")] = "n";
        rot90[("ccw", "n")] = "w";
        rot90[("ccw", "w")] = "s";
        lenVek = new Dictionary<string, Vector3>();
        lenVek["s"] = new Vector3(0, -1, 0);
        lenVek["n"] = new Vector3(0, +1, 0);
        lenVek["e"] = new Vector3(+1, 0, 0);
        lenVek["w"] = new Vector3(-1, 0, 0);
        compassSigns = new Dictionary<string, float>();
        compassSigns["cw"] = 1;
        compassSigns["ccw"] = -1;
        angleval = new Dictionary<string, float>();
        angleval["n"] = 0;
        angleval["e"] = 90;
        angleval["s"] = 180;
        angleval["w"] = 270;
        inited = true;
    }
    [NonSerialized]
    MmPath path;
    GameObject startgo;
    public MmSegForm mmSegForm = MmSegForm.None;
    public string direction = "";
    public float lengthUnits = 0;
    public string rotdir = "";
    public string startCompassPt = "";
    public Vector3 strpt = Vector3.zero;
    public Vector3 endpt = Vector3.zero;
    public Vector3 cenpt = Vector3.zero;
    public float sang = 0;
    public float eang = 0;
    public float spang = 0;
    public float epang = 0;
    public float rad = 0;
    public MmPathSeg(MmPath path, MmSegForm inform, string name, string direction, float lengthUnits)
    {
        if (inform != MmSegForm.Straight)
        {
            Debug.LogError("Straight Form called incorrectly");
            return;
        }
        if (!direction.Contains(direction))
        {
            Debug.LogError($"Bad MmPathSeg parameter - direction:{direction}");
            return;
        }
        this.name = name;
        this.path = path;
        mmSegForm = MmSegForm.Straight;
        this.direction = direction;
        this.lengthUnits = lengthUnits;
        this.strpt = path.End();
        this.endpt = this.strpt + lenVek[direction] * lengthUnits;
        spang = angleval[direction];
        epang = spang;
        path.AdjustEndPoint(this.endpt);
    }
    public MmPathSeg(MmPath path, MmSegForm inform, string name, string startCompassPt, string rotdir)
    {
        if (inform != MmSegForm.Curved)
        {
            Debug.LogError("Straight Form called incorrectly");
            return;
        }

        if (!compassPoints.Contains(startCompassPt))
        {
            Debug.LogError($"Bad MmPathSeg parameter - startCompassPt:{startCompassPt}");
            return;
        }
        if (!compassDirections.Contains(rotdir))
        {
            Debug.LogError($"Bad MmPathSeg parameter - rotdir:{rotdir}");
            return;
        }
        this.name = name;
        this.path = path;
        mmSegForm = MmSegForm.Curved;
        this.startCompassPt = startCompassPt;
        this.rotdir = rotdir;
        strpt = path.End();
        //this.center = startpt + radVek[startCompassPt];
        cenpt = strpt - lenVek[startCompassPt];
        sang = angleval[startCompassPt];
        spang = sang + compassSigns[rotdir] * 90f;
        eang = sang + compassSigns[rotdir] * 90f;
        epang = eang + compassSigns[rotdir] * 90f; ;
        this.lengthUnits = Mathf.PI / 2f;// 1 quarter circle
        rad = 1f;
        var r9sp = rot90[(rotdir, startCompassPt)];
        //Debug.Log($"startCompasPt:{startCompassPt} rot90:{r9sp}spang:{spang} epang:{epang}");

        var ldir = rot90[(rotdir, startCompassPt)];
        this.endpt = this.strpt - lenVek[startCompassPt] + lenVek[ldir];
        //this.endpt = this.startpt + radVek[startCompassPt] + lenVek[ldir];
        path.AdjustEndPoint(this.endpt);
    }
    float angOfLmb(float lamb)
    {
        var ang = (float)Math.PI * (lamb * (eang - sang) + sang) / 180f;
        return ang;
    }
    public float angOfLmbDeg(float lamb)
    {
        var ang = (float)(lamb * (eang - sang) + sang);
        return ang;
    }
    public float pangOfLmbDeg(float lamb)
    {
        var pang = (float)(lamb * (epang - spang) + spang);
        return pang;
    }
    public Vector3 ptOfLmb(float lamb)
    {
        if (this.mmSegForm == MmSegForm.Curved)
        {
            var ang = angOfLmb(lamb);
            var y = Mathf.Cos(ang) * rad;
            var x = Mathf.Sin(ang) * rad;
            var pt = cenpt + new Vector3(x, y, 0);
            return pt;
        }
        else
        {
            var pt = lamb * (endpt - strpt) + strpt;
            return pt;
        }
    }
    public void MakeGos(GameObject parent)
    {
        startgo = UnityUt.CreateSphere(parent,"green", size: sphrad);
        startgo.name = name;
        startgo.transform.position = strpt;
        startgo = UnityUt.CreateSphere(parent, "blue", size: sphrad);
        startgo.name = name;
        startgo.transform.position = endpt;


        if (this.mmSegForm == MmSegForm.Curved)
        {
            startgo = UnityUt.CreateSphere(parent, "magenta", size: sphrad);
            startgo.name = name + "-center";
            startgo.transform.position = this.cenpt;
            var npts = 10;
            for (int i = 0; i < npts; i++)
            {
                var frac = i * 1.0f / npts;
                var ang = angOfLmbDeg(frac);
                var pt = ptOfLmb(frac);

                var go = UnityUt.CreateSphere(parent,"white", size: sphrad / 2);
                go.name = $"{name} ang:{ang:f0} frac:{frac:f2}";
                //go.transform.position = UnitsToMeters(pt);
                go.transform.position = pt;
            }
        }
        else
        {
            var npts = 10;
            for (int i = 0; i < npts; i++)
            {
                var frac = i * 1.0f / npts;
                var pt = frac * (endpt - strpt) + strpt;
                var go = UnityUt.CreateSphere(parent,"gray", size: sphrad / 2);
                go.name = $"{name} line frac:{frac:f2}";
                //go.transform.position = UnitsToMeters(pt);
                go.transform.position = pt;
            }
        }
    }
}

[Serializable]
public class MmPath
{
    static float sphrad = 0.6f;
    [NonSerialized]
    MmTable mmt;
    GameObject startgo;
    public string name;
    Vector3 startpt;
    Vector3 endpt;
    public int pidx;
    public float unitLength = 0;
    public List<MmPathSeg> segs = new List<MmPathSeg>();
    [NonSerialized]
    public List<MmPath> continuationPaths = new List<MmPath>();

    public MmPath(MmTable mmt, int idx, string name, Vector3 startpt)
    {
        this.mmt = mmt;
        this.name = name;
        this.pidx = idx;
        this.startpt = startpt;
        this.endpt = this.startpt;
    }

    //public MmRail MakePathRail0(string rname, int pathnum, float pathdist)
    //{
    //    var railgo = new GameObject(rname);
    //    var (pt, ang) = mmt.GetPositionAndOrientation(pathnum,pathdist);
    //    railgo.transform.position = pt;
    //    railgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
    //    var rail = railgo.AddComponent<MmRail>();
    //    var railform = MmRail.RailForm.Cigar;
    //    var sledid = "";
    //    rail.ConstructForm(mmt, railgo, railform, sledid, pathnum, pathdist, speed: 0);
    //    railgo.transform.parent = mmt.mmtgo.transform;
    //    return rail;
    //}

    public MmRail MakePathRail(string sledid, int pathnum, float pathdist)
    {
        var rail = MmRail.ConstructRail(this.mmt, sledid, pathnum, pathdist);
        mmt.rails.Add(rail);
        return rail;
    }
    public void AddPathRails(GameObject parent, bool seggos, bool pathgos)
    {

        var sz = sphrad / 2;
        var pos = this.startpt;
        if (mmt.useMeters)
        {
            sz *= mmt.UnitsToMeters;
            pos *= mmt.UnitsToMeters;
        }
        startgo = UnityUt.CreateSphere(parent,"red", size: sz);
        startgo.name = name;
        startgo.transform.position = pos;
        if (seggos)
        {
            foreach (var seg in segs)
            {
                seg.MakeGos(startgo);
            }
        }
        if (pathgos)
        {
            var nrails = (int)this.unitLength / 0.4f;
            for (int i = 0; i < nrails; i++)
            {
                var frac = i * 1.0f / nrails;
                var pathdist = frac * this.unitLength;
                var (pt, ang) = GetPositionAndOrientation(pathdist);
                var rname = $"{name} rail - frac:{frac:f2} ang:{ang:f0}";
                MakePathRail(rname, pidx, pathdist);
            }
        }
    }
    int selcount = 0;
    public (int newpathidx, float newpathdist, bool markfordeletion) AdvancePathdist(float curpathdist, float deltadist)
    {
        var newdist = curpathdist + deltadist;
        if (newdist < this.unitLength)
        {
            return (pidx, newdist, false);
        }
        var restdist = newdist - this.unitLength;
        if (continuationPaths.Count == 0)
        {
            return (pidx, this.unitLength, true);
        }
        var newpath = continuationPaths[selcount % continuationPaths.Count];
        selcount++;
        return (newpath.pidx, restdist, false);
    }

    public void LinkToContinuationPath(MmPath contpath)
    {
        continuationPaths.Add(contpath);
    }
    public void AdjustEndPoint(Vector3 newendpoint)
    {
        this.endpt = newendpoint;
    }
    public Vector3 End()
    {
        return endpt;
    }
    public void MakeLineSeg(string direction, float lengthUnits)
    {
        var name = $"line-seg {segs.Count}";
        var seg = new MmPathSeg(this, MmSegForm.Straight, name, direction, lengthUnits);
        unitLength += seg.lengthUnits;
        segs.Add(seg);

    }
    public void MakeCircSeg(string startCompassPt, string rotdir)
    {
        var name = $"circ-seg {segs.Count}";
        var seg = new MmPathSeg(this, MmSegForm.Curved, name, startCompassPt, rotdir);
        unitLength += seg.lengthUnits;
        segs.Add(seg);
    }
    public (Vector3 pt, float ang) GetPositionAndOrientation(float pathdist)
    {
        //Debug.Log($"Pathidx:{pidx} pathdist:{pathdist:f2}");
        if (segs.Count <= 0)
        {
            Debug.LogError($"no segs defined for path {name}");
            return (Vector3.zero, 0);
        }
        if (this.unitLength < pathdist)
        {
            Debug.LogWarning($"pathdist requested is bigger than pathlength for path {name}");
            var eang = segs[segs.Count - 1].eang;
            return (endpt, eang);
        }
        //Debug.Log($"{name} unitLength:{unitLength}");
        var i = 0;
        var sg = segs[i];
        var lo = 0f;
        var hi = sg.lengthUnits;
        while (hi < pathdist)
        {
            //var msg = $"   i:{i} hi:{hi} {lo}";
            //Debug.Log(msg);
            i++;
            sg = segs[i];
            lo = hi;
            hi = hi + sg.lengthUnits;
        }
        var lamb = (pathdist - lo) / sg.lengthUnits;
        //var msg1 = $"   fin-i:{i} hi:{hi} {lo}  lamb:{lamb}";
        //Debug.Log(msg1);
        var pt = sg.ptOfLmb(lamb);
        var ang = sg.pangOfLmbDeg(lamb);
        //Debug.Log($"rv:{rv}");
        if (mmt.useMeters)
        {
            var u2m = mmt.UnitsToMeters;
            pt = new Vector3(u2m * pt.x, u2m * pt.y, u2m * pt.z);
        }
        //Debug.Log($"Pathidx:{pidx} pathdist:{pathdist:f2} returns - pt:{pt:f1} ang:{ang:f1}");
        return (pt, ang);
    }
}

public class MmTable
{
    public GameObject mmtgo;
    public string tableName = "TableName";
    public List<MmPath> paths = new List<MmPath>();
    public float UnitsToMeters = 0.125f;
    public GameObject sledsgo;
    public List<MmSled> sleds = new List<MmSled>();
    public List<MmRail> rails = new List<MmRail>();

    public bool useMeters = false;
    public float sledSpeed = 0.01f;
    public bool interpolateOnSpeed = false;

    public Dictionary<string, MmSled> SledDict = new Dictionary<string, MmSled>();


    public MmTable()
    {
        if (!MmPathSeg.inited)
        {
            MmPathSeg.InitDicts();
        }
    }
    public MmPath GetPath(int idx)
    {
        if (idx < 0)
        {
            Debug.LogError("GetPath - path idx<0");
            return null;
        }
        if (idx >= paths.Count)
        {
            Debug.LogError("GetPath - path idx exceeds count");
            return null;
        }
        return paths[idx];
    }

    public (Vector3 pt, float ang) GetPositionAndOrientation(int pathnum, float pathdist)
    {
        var path = GetPath(pathnum);
        var (pt, ang) = path.GetPositionAndOrientation(pathdist);
        return (pt, ang);
    }


    public void MakeMsftDemoMagmo()
    {
        Debug.Log("Making MsftDemoMagmo");
        tableName = "MsftDemoMagmo";
        var mmt = this;
        var ptstar = new Vector3(4, 0, 0);
        var p1 = mmt.makePath("path1", ptstar);
        p1.MakeLineSeg("w", 2);
        p1.MakeLineSeg("w", 8);
        p1.MakeLineSeg("w", 2);
        p1.MakeCircSeg("s", "cw");
        p1.MakeCircSeg("w", "cw");

        var p2 = mmt.makePath("path2", p1.End());
        p2.MakeCircSeg("s", "ccw");
        p1.LinkToContinuationPath(p2);


        var p3 = mmt.makePath("path3", p2.End());
        p3.MakeLineSeg("n", 2);
        //p2.LinkToContinuationPath(p3);

        var p4 = mmt.makePath("path4", p2.End());
        p4.MakeCircSeg("w", "cw");
        p4.MakeLineSeg("e", 2);
        p4.MakeLineSeg("e", 8);
        p4.MakeLineSeg("e", 2);
        p4.MakeCircSeg("n", "cw");
        p4.MakeCircSeg("w", "ccw");
        p2.LinkToContinuationPath(p4);

        var p5 = mmt.makePath("path5", p1.End());
        p5.MakeLineSeg("e", 2);
        p5.MakeLineSeg("e", 8);
        p5.MakeLineSeg("e", 2);
        p1.LinkToContinuationPath(p5);

        var p6 = mmt.makePath("path6", p5.End());
        p6.MakeLineSeg("e", 2);
        p6.MakeLineSeg("e", 2);
        p5.LinkToContinuationPath(p6);

        var p7 = mmt.makePath("path7", p4.End());
        p7.MakeCircSeg("n", "cw");
        p7.MakeCircSeg("e", "cw");
        p7.MakeLineSeg("w", 2);
        p7.MakeLineSeg("w", 2);
        p4.LinkToContinuationPath(p7);
        p6.LinkToContinuationPath(p7);


        var p8 = mmt.makePath("path8", p7.End());
        p8.MakeCircSeg("s", "cw");
        p8.MakeCircSeg("w", "cw");
        p7.LinkToContinuationPath(p1);
        p7.LinkToContinuationPath(p8);
        p8.LinkToContinuationPath(p6);
    }

    public void MakeSimplePath()
    {
        Debug.Log("Making MsftDemoMagmo");
        tableName = "MsftDemoMagmo";
        var mmt = this;
        var ptstar = new Vector3(0, 0, 0);
        var p1 = mmt.makePath("path1", ptstar);
        p1.MakeLineSeg("n", 8);

        var p2 = mmt.makePath("path2", p1.End());
        p2.MakeCircSeg("w", "cw");
        p2.MakeLineSeg("e", 8);
        p2.MakeCircSeg("n", "cw");
        p2.MakeLineSeg("s", 8);
        p1.LinkToContinuationPath(p2);

        var p3 = mmt.makePath("path3", p2.End());
        p3.MakeCircSeg("e", "cw");
        p3.MakeLineSeg("w", 8);
        p3.MakeCircSeg("s", "cw");
        p2.LinkToContinuationPath(p3);
        p3.LinkToContinuationPath(p1);
    }


    //public MmSled ConstructSled(string sledid, int pathnum, float pathdist, bool loaded)
    //{
    //    var sname1 = $"sledid:{sledid}";
    //    var sledgeomgo = new GameObject(sname1);
    //    var (pt, ang) = GetPositionAndOrientation(pathnum, pathdist);
    //    sledgeomgo.transform.position = pt;
    //    sledgeomgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
    //    var sled = sledgeomgo.AddComponent<MmSled>();
    //    var sledform = MmSled.SledForm.Prefab;
    //    sled.ConstructForm(this, sledgeomgo, sledform, sledid, pathnum, pathdist, loaded);
    //    sledgeomgo.transform.parent = this.mmtgo.transform;
    //    return sled;
    //    //Debug.Log($"makesled pathnum:{pathnum} dist:{pathdist:f1} pt:{sledgeomgo.transform.position:f1}");
    //}


    public MmSled MakeSled(string sledid, int pathnum, float pathdist, bool loaded)
    {
        var sled = MmSled.ConstructSled(this,sledid, pathnum, pathdist, loaded);
        this.sleds.Add(sled);
        this.SledDict[sledid] = sled;
        return sled;
    }


    MmPath makePath(string name, Vector3 pt)
    {
        var idx = paths.Count;
        var rv = new MmPath(this, idx, name, pt);
        paths.Add(rv);
        return rv;
    }
    public GameObject SetupGeometry(bool addPathMarkers, bool addPathSleds, bool positionOnFloor)
    {
        mmtgo = new GameObject(tableName);

        if (addPathMarkers)
        {
            foreach (var p in paths)
            {
                p.AddPathRails(mmtgo, seggos: false, pathgos: true);
            }
        }

        // Add Sleds
        if (addPathSleds)
        {
            var totsleds = 0;
            foreach (var p in paths)
            {
                if (p.continuationPaths.Count > 0) // no dead edns
                {
                    var nsleds = (int)p.unitLength / 3.0f;
                    for (int i = 0; i < nsleds; i++)
                    {
                        if (totsleds <= 10) // only create 10
                        {
                            var frac = i * 1.0f / nsleds;
                            var pathdist = frac * p.unitLength;
                            var iid = sleds.Count + 1;
                            var sledid = $"{iid}";
                            MakeSled(sledid, p.pidx, pathdist, loaded: true);
                            totsleds++;
                        }
                    }
                }
            }
        }
        if (positionOnFloor)
        {
            // flatten to XZ plane and twist around
            mmtgo.transform.localRotation = Quaternion.Euler(90, 180, 0);

            // attach to floor if there is one
            var floorgo = GameObject.Find("Floor");
            if (floorgo != null)
            {
                mmtgo.transform.SetParent(floorgo.transform, worldPositionStays: false);
                // move it behind the robot and up to the first robot joint 
                mmtgo.transform.position += new Vector3(0.2f, 0.2f, 0.77f);
            }
        }
        return mmtgo;
    }

    public void DeleteSledsAsNeeded()
    {
        var deleteList = new List<MmSled>();
        foreach (var sled in sleds)
        {
            if (sled.ShouldDeleted())
            {
                deleteList.Add(sled);
            }
        }
        if (deleteList.Count > 0)
        {
            Debug.Log($"Deleteing {deleteList.Count} sleds");
            foreach (var sled in deleteList)
            {
                if (SledDict.ContainsKey(sled.sledid))
                {
                    SledDict.Remove(sled.sledid);
                }
                Debug.Log($"   Deleteing {sled.name} ");
                sleds.Remove(sled);
                sled.DeleteStuff();
            }
            Debug.Log($"{sleds.Count} sleds left");
        }
    }

    System.Random ran = new System.Random(1234);
    float lastLoadChange = 0f;
    float timeToLoadStateChange = 3f;
    public void UpdateTable()
    {
        foreach (var sled in sleds)
        {
            sled.AdvanceSledBySpeed();
        }
    }
}

public class MagneMotion : MonoBehaviour
{
    public bool useMeters = false;
    public bool addPathMarkers = false;
    public bool addPathSledsOnStartup = true;
    public bool positionOnFloor = false;
    public MmTable mmtable = null;
    public MmTray mmtray = null;
    // Start is called before the first frame update
    void Start()
    {
        mmtable = new MmTable();
        mmtable.useMeters = useMeters;
        mmtable.MakeMsftDemoMagmo();
        //mmtable.MakeSimplePath();
        var mmgo = mmtable.SetupGeometry(addPathMarkers: addPathMarkers, addPathSleds: addPathSledsOnStartup, positionOnFloor: positionOnFloor);
        mmgo.transform.SetParent(gameObject.transform, false);

        mmtray = FindObjectOfType<MmTray>();
        if (mmtray != null)
        {
            mmtray.Init(mmtable);
        }
        ROSConnection.GetOrCreateInstance().Subscribe<MmSledMsg>("Rs007Sleds", SledChange);
    }


    void SledChange(MmSledMsg sledmsg)
    {
        // Debug.Log($"Received ROS message on topic Rs007Sleds:{sledmsg.ToString()}");
        var sledid = $"{sledmsg.cartid}";
        var loaded = sledmsg.loaded;
        var pathid = sledmsg.pathid - 1;
        var position = (float)sledmsg.position / mmtable.UnitsToMeters;
        if (sledmsg.pathid < 0)
        {
            Debug.LogWarning($"Bad pathid detected {sledmsg.pathid} on cartid:{sledmsg.cartid}");
            return;
        }
        if (mmtable.SledDict.ContainsKey(sledid))
        {
            var sled = mmtable.SledDict[sledid];
            var oldstate = sled.GetLoadState();
            sled.UpdateSled(pathid, position, loaded);
            if (oldstate != loaded)
            {
                Debug.Log($"Sled {sledid} changed loaded state to {loaded}");
            }
        }
        else
        {
            //Debug.Log($"making sled:{sledid}");
            if (pathid >= 0)
            {
                mmtable.MakeSled(sledid, pathid, position, loaded);
            }
        }
    }
    // Update is called once per frame
    void Update()
    {
        mmtable.DeleteSledsAsNeeded();
        mmtable.UpdateTable();
    }
}
