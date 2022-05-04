using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MmRail : MonoBehaviour
{
    MmTable mmt;
    public enum RailForm { Cigar,Box }
    static float sphrad = 0.2f;
    float speed;
    int pathnum;
    float pathdist;
    RailForm railform;
    GameObject geomgo;
    GameObject formgo;
    public string sledid;
    // Start is called before the first frame update
    void Start()
    {
    }

    public void Construct(MmTable mmt,GameObject geomgo, RailForm railform, string sledid,int pathnum,float pathdist,  float speed=0)
    {
        this.geomgo = geomgo;
        this.mmt = mmt;
        this.railform = railform;
        this.speed = speed;
        formgo = new GameObject("railform");
        this.pathnum = pathnum;
        this.pathdist = pathdist;
        this.sledid = sledid;
        var path = mmt.GetPath(pathnum);
        var (pt,ang) = path.GetPositionAndOrientation(pathdist);
        switch (this.railform)
        {
            case RailForm.Cigar:
                {
                    MmUtil.mmcolor = Color.gray;
                    var go = MmUtil.CreateSphere(formgo, size: sphrad / 3);
                    go.name = $"cigar";
                    go.transform.localScale = new Vector3(0.1f, 0.3f, 0.1f);

                    MmUtil.mmcolor = Color.red;
                    var go1 = MmUtil.CreateSphere(formgo, size: sphrad / 3);
                    go1.name = $"nose";
                    go1.transform.position = new Vector3(0.0f, 0.1f, 0);
                    go1.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                    break;
                }
            case RailForm.Box:
                {
                    MmUtil.mmcolor = Color.gray;
                    var go = MmUtil.CreateCube(formgo, size: sphrad / 3);
                    go.name = $"tray";
                    // 6.5x11.0x2cm
                    go.transform.localScale = new Vector3(0.88f, 0.52f, 0.16f);


                    MmUtil.mmcolor = MmUtil.GetRandomColor();
                    var go2 = MmUtil.CreateSphere(formgo, size: sphrad / 3);
                    go2.name = $"nose";
                    go2.transform.position = new Vector3(0.0f, 0.2f, -0.16f);
                    go2.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                    break;
                }

        }
        formgo.transform.position = pt;
        formgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
        if (mmt.useMeters)
        {
            var u2m = mmt.UnitsToMeters;
            formgo.transform.localScale = new Vector3( u2m,u2m,u2m );
        }
        formgo.transform.parent = geomgo.transform;
        //AdjustSledPositionAndOrientation(pt, ang);
    }




    int updatecount = 0;
    // Update is called once per frame
    void Update()
    {
        updatecount++;

    }
}
