using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public enum MmSegForm {  None, Straight, Curved }


public class MmUtil
{
    public static Color mmcolor = Color.green;
    public static GameObject CreateSphere(GameObject parent,float size=0.5f)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.transform.parent = parent.transform;
        go.transform.localScale = new Vector3(size, size, size);
        var material = go.GetComponent<Renderer>().material;
        material.color = mmcolor;
        return go;
    }
}

[Serializable]
public class MmPathSeg
{
    public static bool inited = false;
    static float sphrad = 0.1f;
    static Dictionary<string, Vector3> radVek;
    static Dictionary<string, Vector3> lenVek;
    static Dictionary<string, Vector3> crcEndVek;
    static Dictionary<string, float> compassSigns;
    static List<string> compassPoints = new List<string>() { "s", "n", "e", "w" };
    static List<string> compassDirections = new List<string>() { "cw", "ccw"  };


    public static void InitDicts()
    {
        Debug.Log("InitDicts");
        radVek = new Dictionary<string, Vector3>();
        radVek["s"] = new Vector3(0, +1, + 0);
        radVek["n"] = new Vector3(0, -1,  0);
        radVek["e"] = new Vector3(-1,  0, 0);
        radVek["w"] = new Vector3(+1,  0, 0);
        lenVek = new Dictionary<string, Vector3>();
        lenVek["s"] = new Vector3(0,  -1,0);
        lenVek["n"] = new Vector3(0,  +1, 0);
        lenVek["e"] = new Vector3(+1, 0, 0);
        lenVek["w"] = new Vector3(-1, 0, 0);
        crcEndVek = new Dictionary<string, Vector3>();
        foreach (var s in compassPoints)
        {
            crcEndVek[s] = Vector3.Normalize(radVek[s] + lenVek[s]); 
        }
        compassSigns = new Dictionary<string, float>();
        compassSigns["cw"] = 1;
        compassSigns["ccw"] = -1;
        inited = true;
    }
    [NonSerialized]
    MmPath path;
    public MmSegForm mmSegForm = MmSegForm.None;
    public string direction="";
    public float lengthCm=0;
    public string rotdir="";
    public string startCompassPt="";
    public Vector3 startpt = Vector3.zero;
    public Vector3 endpt = Vector3.zero;
    public Vector3 center = Vector3.zero;
    public MmPathSeg(MmPath path,string direction,float lengthBlocks)
    {
        if (!direction.Contains(direction))
        {
            Debug.LogError($"Bad MmPathSeg parameter - direction:{direction}");
            return;
        }

        this.path = path;
        mmSegForm = MmSegForm.Straight;
        this.direction = direction;
        this.lengthCm = lengthBlocks*0.125f;
        this.startpt = path.End();
        this.endpt = this.startpt + lenVek[direction] * lengthBlocks;
        path.AdjustEndPoint(this.endpt);
    }
    public MmPathSeg(MmPath path, string startCompassPt, string rotdir)
    {
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

        this.path = path;
        mmSegForm = MmSegForm.Curved;
        this.startCompassPt = startCompassPt;
        this.rotdir = rotdir;
        this.center = path.End() + radVek[startCompassPt] * 0.125f;
        this.startpt = path.End();
        this.endpt = this.startpt + compassSigns[rotdir]*radVek[startCompassPt];
        path.AdjustEndPoint(this.endpt);
    }
}

[Serializable]
public class MmPath
{
    [NonSerialized]
    MmTable mmt;
    GameObject startgo;
    public string name;
    int nstarts = 1;
    Vector3 startpt1;
    Vector3 startpt2;
    Vector3 endpt;
    public List<MmPathSeg> segs = new List<MmPathSeg>();

    public MmPath(MmTable mmt,string name,Vector3 startpt)
    {
        this.mmt = mmt;
        this.name = name;
        nstarts = 1;
        this.startpt1 = startpt;
        this.endpt = startpt1;

    }
    public void MakeGos()
    {
        if (mmt.mmtgo != null)
        {
            startgo = MmUtil.CreateSphere(mmt.mmtgo);
            startgo.name = name;
            startgo.transform.position = this.startpt1;
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
    public void makeStrSeg( string direction, float lengthCm)
    {
        var seg = new MmPathSeg(this, direction, lengthCm);
        segs.Add(seg);
    }
    public void makeCrcSeg(string startCompassPt, string rotdir)
    {
        var seg = new MmPathSeg(this, startCompassPt, rotdir);
        segs.Add(seg);
    }
}

[Serializable]
public class MmTable
{
    public GameObject mmtgo;
    public string tableName="TableName";
    public List<MmPath> paths = new List<MmPath>();

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
        p4.makeCrcSeg("e", "cw");
        p4.makeStrSeg("e", 2);
        p4.makeStrSeg("e", 8);
        p4.makeStrSeg("e", 2);
        p4.makeCrcSeg("n", "cw");
        p4.makeCrcSeg("w", "cw");

        var p5 = mmt.makePath("path5", p1.End());
        p5.makeStrSeg("w", 2);
        p5.makeStrSeg("w", 8);
        p5.makeStrSeg("w", 2);

        var p6 = mmt.makePath("path6", p5.End());
        p6.makeStrSeg("w", 2);
        p6.makeStrSeg("w", 2);

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
            p.MakeGos();
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
