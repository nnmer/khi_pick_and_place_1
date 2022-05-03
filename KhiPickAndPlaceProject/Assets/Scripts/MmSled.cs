using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MmSled : MonoBehaviour
{
    MmTable mmt;
    public enum SledForm { Cigar,Boxes }
    static float sphrad = 0.2f;
    float speed;
    int pathnum;
    float pathdist;
    SledForm sledform;
    GameObject sledgo;
    public string sledid;
    // Start is called before the first frame update
    void Start()
    {
        
    }
    public bool ShouldDelete()
    {
        return pathnum < 0;
    }
    public void DeleteStuff()
    {
        Destroy(sledgo);
    }
    public void Construct(MmTable mmt,GameObject parent, SledForm sledform, string sledid,int pathnum,float pathdist,  float speed=0)
    {
        this.mmt = mmt;
        this.sledform = sledform;
        this.speed = speed;
        sledgo = new GameObject("sled");
        this.pathnum = pathnum;
        this.pathdist = pathdist;
        this.sledid = sledid;
        var path = mmt.GetPath(pathnum);
        var (pt,ang) = path.GetPositionAndOrientation(pathdist);
        switch (this.sledform)
        {
            case SledForm.Cigar:
                {
                    MmUtil.mmcolor = Color.gray;
                    var go = MmUtil.CreateSphere(sledgo, size: sphrad / 3);
                    go.name = $"cigar";
                    go.transform.localScale = new Vector3(0.1f, 0.3f, 0.1f);

                    MmUtil.mmcolor = Color.red;
                    var go1 = MmUtil.CreateSphere(sledgo, size: sphrad / 3);
                    go1.name = $"nose";
                    go1.transform.position = new Vector3(0.0f, 0.1f, 0);
                    go1.transform.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                    break;
                }
            case SledForm.Boxes:
                {
                    MmUtil.mmcolor = Color.gray;
                    var go = MmUtil.CreateCube(sledgo, size: sphrad / 3);
                    go.name = $"tray";
                    // 6.5x11.0x2cm
                    go.transform.localScale = new Vector3(0.88f, 0.52f, 0.16f);

                    MmUtil.mmcolor = Color.yellow;
                    var go1 = MmUtil.CreateCube(sledgo, size: sphrad / 3);
                    go1.name = $"box";
                    // 7x5.4x4.3.5
                    go1.transform.position = new Vector3(0.0f, 0.0f, -0.16f);
                    go1.transform.localScale = new Vector3(0.56f, 0.32f, 0.28f);

                    MmUtil.mmcolor = MmUtil.GetRandomColor();
                    var go2 = MmUtil.CreateSphere(sledgo, size: sphrad / 3);
                    go2.name = $"nose";
                    go2.transform.position = new Vector3(0.0f, 0.2f, -0.16f);
                    go2.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                    break;
                }
        }
        sledgo.transform.position = pt;
        sledgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
        if (mmt.useMeters)
        {
            var u2m = mmt.UnitsToMeters;
            sledgo.transform.localScale = new Vector3( u2m,u2m,u2m );
        }
        sledgo.transform.parent = parent.transform;
    }

    int updatecount = 0;
    // Update is called once per frame
    void Update()
    {
        updatecount++;
        if (speed>0 && pathnum>=0)
        {
            var deltdist = speed * Time.deltaTime;
            var path = mmt.GetPath(pathnum);
            (pathnum, pathdist) = path.AdvancePathdist(pathdist, deltdist);
            var newpath = mmt.GetPath(pathnum);
            var (pt, ang) = newpath.GetPositionAndOrientation(pathdist);
            var parenttrans = sledgo.transform.parent;
            var parentparenttrans = parenttrans.transform.parent;
            parenttrans.transform.parent = null;
            parenttrans.transform.position = pt;
            parenttrans.transform.rotation = Quaternion.Euler(0, 0, -ang);
            parenttrans.transform.SetParent(parentparenttrans, worldPositionStays:false);
        }
        //if (speed > 0 && pathnum >= 0)
        //{
        //    var deltdist = speed * Time.deltaTime;
        //    var path = mmt.GetPath(pathnum);
        //    (pathnum, pathdist) = path.AdvancePathdist(pathdist, deltdist);
        //    var newpath = mmt.GetPath(pathnum);
        //    var (pt, ang) = newpath.GetPositionAndOrientation(pathdist);
        //    var parenttrans = sledgo.transform.parent;
        //    sledgo.transform.parent = null;
        //    sledgo.transform.position = pt;
        //    sledgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
        //    sledgo.transform.SetParent(parenttrans,worldPositionStays:false);
        //}
    }
}
