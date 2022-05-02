using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public enum MmSegForm {  None, Straight, Curved }


public class MmUtil
{
    public static Color mmcolor = Color.white;
    public static GameObject CreateSphere(GameObject parent,float size=0.5f)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.transform.localScale = new Vector3(size, size, size);
        go.transform.parent = parent.transform;
        var material = go.GetComponent<Renderer>().material;
        material.color = mmcolor;
        return go;
    }
    public static GameObject CreateCube(GameObject parent, float size = 0.5f)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.transform.localScale = new Vector3(size, size, size);
        go.transform.parent = parent.transform;
        var material = go.GetComponent<Renderer>().material;
        material.color = mmcolor;
        return go;
    }
    static System.Random random = new System.Random(1234);
    public static Color GetRandomColor()
    {
        var i = random.Next(4);
        switch(i)
        {
            default:
            case 0: return Color.red;
            case 1: return Color.green;
            case 2: return Color.blue;
            case 3: return Color.black;
        }
    }
}

[Serializable]
public class MmPathSeg
{
    string name;
    public static bool inited = false;
    static float sphrad = 0.3f;
    static Dictionary<string,float> angleval;
    static Dictionary<(string,string), string> rot90;
    static Dictionary<string, Vector3> lenVek;
    static Dictionary<string, float> compassSigns;
    static List<string> compassPoints = new List<string>() { "s", "n", "e", "w" };
    static List<string> compassDirections = new List<string>() { "cw", "ccw"  };



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
        lenVek["s"] = new Vector3(0,  -1,0);
        lenVek["n"] = new Vector3(0,  +1, 0);
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
    public string direction="";
    public float lengthUnits = 0;
    public string rotdir="";
    public string startCompassPt="";
    public Vector3 strpt = Vector3.zero;
    public Vector3 endpt = Vector3.zero;
    public Vector3 cenpt = Vector3.zero;
    public float sang = 0;
    public float eang = 0;
    public float spang = 0;
    public float epang = 0;
    public float rad = 0;
    public MmPathSeg(MmPath path,MmSegForm inform,string name,string direction,float lengthUnits)
    {
        if (inform!=MmSegForm.Straight)
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
        epang = spang ;
        path.AdjustEndPoint(this.endpt);
    }
    public MmPathSeg(MmPath path, MmSegForm inform,string name, string startCompassPt, string rotdir)
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
        this.lengthUnits = Mathf.PI/2f;// 1 quarter circle
        rad = 1f;
        var r9sp = rot90[(rotdir, startCompassPt)];
        //Debug.Log($"startCompasPt:{startCompassPt} rot90:{r9sp}spang:{spang} epang:{epang}");

        var ldir = rot90[(rotdir,startCompassPt)];
        this.endpt = this.strpt - lenVek[startCompassPt] + lenVek[ldir];
        //this.endpt = this.startpt + radVek[startCompassPt] + lenVek[ldir];
        path.AdjustEndPoint(this.endpt);
    }
    float angOfLmb(float lamb)
    {
        var ang = (float) Math.PI*(lamb * (eang - sang) + sang)/180f;
        return ang;
    }
    public float angOfLmbDeg(float lamb)
    {
        var ang = (float) (lamb * (eang - sang) + sang);
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
    Vector3 UnitsToMeters(Vector3 v)
    {
        var rv = MmTable.UnitsToMeters * v;
        return rv;
    }
    public void MakeGos(GameObject parent)
    {
        MmUtil.mmcolor = Color.green;
        startgo = MmUtil.CreateSphere(parent, size: sphrad);
        startgo.name = name;
        startgo.transform.position = strpt;
        MmUtil.mmcolor = Color.blue;
        startgo = MmUtil.CreateSphere(parent, size: sphrad);
        startgo.name = name;
        startgo.transform.position = endpt;
        

        if (this.mmSegForm == MmSegForm.Curved)
        {
            MmUtil.mmcolor = Color.magenta;
            startgo = MmUtil.CreateSphere(parent, size: sphrad);
            startgo.name = name + "-center";
            startgo.transform.position = this.cenpt;
            var npts = 10;
            for (int i = 0; i < npts; i++)
            {
                var frac = i * 1.0f / npts;
                var ang = angOfLmbDeg(frac);
                var pt = ptOfLmb(frac);
                MmUtil.mmcolor = Color.white;
                //Color greenColor = new UnityEngine.Color("#0AC742");
                //ColorUtility.TryParseHtmlString("#0AC742", out greenColor);
                //MmUtil.mmcolor = greenColor;

                var go = MmUtil.CreateSphere(parent, size: sphrad / 2);
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
                MmUtil.mmcolor = Color.gray;
                var go = MmUtil.CreateSphere(parent, size:  sphrad / 2);
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
    int idx;
    public float unitLength = 0;
    public List<MmPathSeg> segs = new List<MmPathSeg>();
    [NonSerialized]
    public List<MmPath> continuationPaths=new List<MmPath>();

    public MmPath(MmTable mmt,int idx,string name,Vector3 startpt)
    {
        this.mmt = mmt;
        this.name = name;
        this.idx = idx;
        this.startpt = startpt;
        this.endpt = this.startpt;
    }

    public void MakeSled(string sname,int pathnum,float pathdist,Vector3 pt,float ang)
    {
        var sledgo = new GameObject(sname);
        sledgo.transform.position = pt;
        sledgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
        var sled = sledgo.AddComponent<MmSled>();
        mmt.sleds.Add(sled);
        var sledform = MmSled.SledForm.Boxes;
        sled.Construct(mmt,sledgo, sledform, sname,pathnum,pathdist, pt, ang, 2.0f);
        sledgo.transform.parent = mmt.sledsgo.transform;
    }
    public void MakeRail(string sname, int pathnum, float pathdist, Vector3 pt, float ang)
    {
        var railgo = new GameObject(sname);
        railgo.transform.position = pt;
        railgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
        var rail = railgo.AddComponent<MmSled>();
        mmt.rails.Add(rail);
        var sledform = MmSled.SledForm.Cigar;
        rail.Construct(mmt,railgo, sledform, sname, pathnum, pathdist, pt, ang);
        railgo.transform.parent = mmt.sledsgo.transform;
    }
    public void MakeGos(GameObject parent,bool seggos,bool pathgos)
    {
        MmUtil.mmcolor = UnityEngine.Color.red;

        startgo = MmUtil.CreateSphere(parent, size: sphrad/2);
        startgo.name = name;
        startgo.transform.position = this.startpt;
        if (seggos)
        {
            foreach (var seg in segs)
            {
                seg.MakeGos(startgo);
            }
        }
        if (pathgos)
        {
            // sleds
            var nsleds = (int) this.unitLength/3.0f;
            for (int i = 0; i < nsleds; i++)
            {
                var frac = i * 1.0f / nsleds;
                var pathdist = frac * this.unitLength;
                var (pt,ang) = GetPositionAndOrientation(pathdist);
                var sname = $"sled-path:{idx}-{i}";
                MakeSled(sname,idx,pathdist,pt, ang);
            }
            // rails
            var nrails = (int)this.unitLength /0.4f;
            for (int i = 0; i < nrails; i++)
            {
                var frac = i * 1.0f / nrails;
                var pathdist = frac * this.unitLength;
                var (pt, ang) = GetPositionAndOrientation(pathdist);
                var rname = $"{name} rail - frac:{frac:f2} ang:{ang:f0}";
                MakeRail(rname,idx,pathdist, pt, ang);
            }
        }
    }
    int selcount = 0;
    public (int newpathidx,float newpathdist) AdvancePathdist(float curpathdist,float deltadist)
    {
        var newdist = curpathdist + deltadist;
        if (newdist < this.unitLength)
        {
            return (idx, newdist);
        }
        var restdist = newdist - this.unitLength;
        if (continuationPaths.Count==0)
        {
            return (-1, this.unitLength);
        }
        var newpath = continuationPaths[selcount % continuationPaths.Count];
        selcount++;
        return (newpath.idx, restdist);
    }

    public void AddContinuationPath(MmPath contpath)
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
    public void makeStrSeg( string direction, float lengthUnits)
    {
        var name= $"line-seg {segs.Count}";
        var seg = new MmPathSeg(this, MmSegForm.Straight, name, direction, lengthUnits);
        unitLength += seg.lengthUnits;
        segs.Add(seg);

    }
    public void makeCrcSeg(string startCompassPt, string rotdir)
    {
        var name = $"circ-seg {segs.Count}";
        var seg = new MmPathSeg(this, MmSegForm.Curved, name, startCompassPt, rotdir);
        unitLength += seg.lengthUnits;
        segs.Add(seg);
    }
    public (Vector3,float) GetPositionAndOrientation(float pathdist)
    {
        //Debug.Log($"GetPosition:{pathdist}");
        if (segs.Count <= 0)
        {
            Debug.LogError($"no segs defined for path {name}");
            return (Vector3.zero,0);
        }
        if (this.unitLength<pathdist)
        {
            Debug.LogWarning($"pathdist requested is bigger than pathlength for path {name}");
            var eang = segs[segs.Count - 1].eang;
            return (endpt,eang);
        }
        //Debug.Log($"{name} unitLength:{unitLength}");
        var i = 0;
        var sg = segs[i];
        var lo = 0f;
        var hi = sg.lengthUnits;
        while (hi<pathdist)
        {
            //var msg = $"   i:{i} hi:{hi} {lo}";
            //Debug.Log(msg);
            i++;
            sg = segs[i];
            lo = hi;
            hi = hi+sg.lengthUnits;
        }
        var lamb = (pathdist - lo) / sg.lengthUnits;
        //var msg1 = $"   fin-i:{i} hi:{hi} {lo}  lamb:{lamb}";
        //Debug.Log(msg1);
        var pt = sg.ptOfLmb(lamb);
        var ang = sg.pangOfLmbDeg(lamb);
        //Debug.Log($"rv:{rv}");
        return (pt,ang);
    }
}

public class MmTable
{
    public GameObject mmtgo;
    public string tableName = "TableName";
    public List<MmPath> paths = new List<MmPath>();
    public static float UnitsToMeters = 0.125f;
    public GameObject sledsgo;
    public List<MmSled> sleds = new List<MmSled>();
    public List<MmSled> rails = new List<MmSled>();

    public MmTable()
    {
        if (!MmPathSeg.inited)
        {
            MmPathSeg.InitDicts();
        }
    }
    public MmPath GetPath(int idx)
    {
        return paths[idx];
    }
    public void MakeMsftDemoMagmo()
    {
        Debug.Log("Making MsftDemoMagmo");
        tableName = "MsftDemoMagmo";
        var mmt = this;
        var ptstar = new Vector3(4, 0, 0);
        var p1 = mmt.makePath("path1", ptstar);
        p1.makeStrSeg("w", 2);
        p1.makeStrSeg("w", 8);
        p1.makeStrSeg("w", 2);
        p1.makeCrcSeg("s", "cw");
        p1.makeCrcSeg("w", "cw");

        var p2 = mmt.makePath("path2", p1.End());
        p2.makeCrcSeg("s", "ccw");
        p1.AddContinuationPath(p2);


        var p3 = mmt.makePath("path3", p2.End());
        p3.makeStrSeg("n", 2);
        p2.AddContinuationPath(p3);

        var p4 = mmt.makePath("path4", p2.End());
        p4.makeCrcSeg("w", "cw");
        p4.makeStrSeg("e", 2);
        p4.makeStrSeg("e", 8);
        p4.makeStrSeg("e", 2);
        p4.makeCrcSeg("n", "cw");
        p4.makeCrcSeg("w", "ccw");
        p2.AddContinuationPath(p4);

        var p5 = mmt.makePath("path5", p1.End());
        p5.makeStrSeg("e", 2);
        p5.makeStrSeg("e", 8);
        p5.makeStrSeg("e", 2);
        p1.AddContinuationPath(p5);

        var p6 = mmt.makePath("path6", p5.End());
        p6.makeStrSeg("e", 2);
        p6.makeStrSeg("e", 2);
        p5.AddContinuationPath(p6);

        var p7 = mmt.makePath("path7", p4.End());
        p7.makeCrcSeg("n", "cw");
        p7.makeCrcSeg("e", "cw");
        p7.makeStrSeg("w", 2);
        p7.makeStrSeg("w", 2);
        p4.AddContinuationPath(p7);
        p6.AddContinuationPath(p7);


        var p8 = mmt.makePath("path8", p7.End());
        p8.makeCrcSeg("s", "cw");
        p8.makeCrcSeg("w", "cw");
        p7.AddContinuationPath(p1);
        p7.AddContinuationPath(p8);
        p8.AddContinuationPath(p6);
    }

    MmPath makePath(string name,Vector3 pt)
    {
        var idx = paths.Count;
        var rv = new MmPath(this, idx, name, pt);
        paths.Add(rv);
        return rv;
    }
    public void MakeGos()
    {
        mmtgo = new GameObject(tableName);
        sledsgo = new GameObject("Sleds");
        sledsgo.transform.parent = mmtgo.transform;
        foreach (var p in paths)
        {
            p.MakeGos(mmtgo,seggos:false,pathgos:true);
        }
    }
}

public class MagneMotion : MonoBehaviour
{
    public MmTable mmt=null;
    // Start is called before the first frame update
    void Start()
    {
        mmt = new MmTable();
        mmt.MakeMsftDemoMagmo();
        mmt.MakeGos();
    }

    // Update is called once per frame
    void Update()
    {
        var deleteList = new List<MmSled>();
        foreach(var sled in mmt.sleds)
        {
            if (sled.ShouldDelete())
            {
                deleteList.Add(sled);
            }
        }
        if (deleteList.Count > 0)
        {
            Debug.Log($"Deleteing {deleteList.Count} sleds");
            foreach (var sled in deleteList)
            {
                Debug.Log($"   Deleteing {sled.name} ");
                mmt.sleds.Remove(sled);
                sled.DeleteStuff();
                Destroy(sled);
            }
            Debug.Log($"{mmt.sleds.Count} sleds left");
        }

    }
}
