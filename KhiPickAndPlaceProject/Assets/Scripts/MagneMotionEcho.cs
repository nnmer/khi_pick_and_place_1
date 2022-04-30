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
        eang = sang + compassSigns[rotdir] *90f;
        this.lengthUnits = Mathf.PI/2f;// 1 quarter circle
        rad = 1f;
        

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
    float angOfLmbDeg(float lamb)
    {
        var ang = (float) (lamb * (eang - sang) + sang);
        return ang;
    }
    Vector3 ptOfLmb(float lamb)
    {
        var ang = angOfLmb(lamb);
        var y = Mathf.Cos(ang) * rad;
        var x = Mathf.Sin(ang) * rad;
        var rv = cenpt + new Vector3(x, y, 0);
        return rv;
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
        var npts = 10;
        

        if (this.mmSegForm == MmSegForm.Curved)
        {
            MmUtil.mmcolor = Color.magenta;
            startgo = MmUtil.CreateSphere(parent, size: sphrad);
            startgo.name = name + "-center";
            startgo.transform.position = this.cenpt;
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
    int nstarts = 1;
    Vector3 startpt1;
    Vector3 startpt2;
    Vector3 endpt;
    public float unitLength = 0;
    public List<MmPathSeg> segs = new List<MmPathSeg>();

    public MmPath(MmTable mmt,string name,Vector3 startpt)
    {
        this.mmt = mmt;
        this.name = name;
        nstarts = 1;
        this.startpt1 = startpt;
        this.endpt = startpt1;

    }
    public void MakeGos(GameObject parent)
    {
        MmUtil.mmcolor = UnityEngine.Color.red;
        startgo = MmUtil.CreateSphere(parent, size: sphrad);
        startgo.name = name;
        startgo.transform.position = this.startpt1;
        foreach(var seg in segs)
        {
            seg.MakeGos(startgo);
        }
    }

    public void AddEndPathPoint(Vector3 newendpathpoint)
    {
        nstarts = 2;
        this.startpt2 = newendpathpoint;
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
}

[Serializable]
public class MmTable
{
    public GameObject mmtgo;
    public string tableName="TableName";
    public List<MmPath> paths = new List<MmPath>();
    public static float UnitsToMeters = 0.125f;
    public MmTable()
    {
        if (!MmPathSeg.inited)
        {
            MmPathSeg.InitDicts();
        }
        MakeRockwellTable();
    }
    public void MakeRockwellTable()
    {
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

        var p3 = mmt.makePath("path3", p2.End());
        p3.makeStrSeg("n", 2);

        var p4 = mmt.makePath("path4", p2.End());
        p4.makeCrcSeg("w", "cw");
        p4.makeStrSeg("e", 2);
        p4.makeStrSeg("e", 8);
        p4.makeStrSeg("e", 2);
        p4.makeCrcSeg("n", "cw");
        p4.makeCrcSeg("w", "ccw");

        var p5 = mmt.makePath("path5", p1.End());
        p5.makeStrSeg("e", 2);
        p5.makeStrSeg("e", 8);
        p5.makeStrSeg("e", 2);

        var p6 = mmt.makePath("path6", p5.End());
        p6.makeStrSeg("e", 2);
        p6.makeStrSeg("e", 2);

        var p7 = mmt.makePath("path7", p4.End());
        p7.AddEndPathPoint(p6.End());
        p7.makeCrcSeg("n", "cw");
        p7.makeCrcSeg("e", "cw");
        p7.makeStrSeg("w", 2);
        p7.makeStrSeg("w", 2);

        var p8 = mmt.makePath("path8", p7.End());
        p8.makeCrcSeg("s", "cw");
        p8.makeCrcSeg("w", "cw");
        p6.AddEndPathPoint(p8.End());
    }

    MmPath makePath(string name, Vector3 pt)
    {
        var rv = new MmPath(this, name, pt);
        paths.Add(rv);
        return rv;
    }
    public void MakeGos()
    {
        mmtgo = new GameObject(tableName);
        foreach(var p in paths)
        {
            p.MakeGos(mmtgo);
        }
    }
}

public class MagneMotionEcho : MonoBehaviour
{
    public MmTable mmt=null;
    // Start is called before the first frame update
    void Start()
    {
        mmt = new MmTable();
        mmt.MakeGos();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
