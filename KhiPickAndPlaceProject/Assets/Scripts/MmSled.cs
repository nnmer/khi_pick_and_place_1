using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MmSled : MonoBehaviour
{
    MmTable mmt;
    public enum SledForm { Cigar,BoxCubeBased,Prefab }
    static float sphrad = 0.2f;
    float speed;
    int pathnum;
    float pathdist;
    bool markedForDeletion = false;
    SledForm sledform;
    GameObject geomgo;
    GameObject formgo;
    public string sledid;
    // Start is called before the first frame update
    void Start()
    {
    }
    public bool ShouldDeleted()
    {
        return markedForDeletion;
    }
    public void MarkForDeletion()
    {
        markedForDeletion = true;
    }
    public void DeleteStuff()
    {
        var parentgo = formgo.transform.parent.gameObject;
        Destroy(geomgo);
    }
    public void Construct(MmTable mmt,GameObject geomgo, SledForm sledform, string sledid,int pathnum,float pathdist,  float speed=0)
    {
        this.geomgo = geomgo;
        this.mmt = mmt;
        this.sledform = sledform;
        this.speed = speed;
        formgo = new GameObject("sled");
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
            case SledForm.BoxCubeBased:
                {
                    MmUtil.mmcolor = Color.gray;
                    var go = MmUtil.CreateCube(formgo, size: sphrad / 3);
                    go.name = $"tray";
                    // 6.5x11.0x2cm
                    go.transform.localScale = new Vector3(0.88f, 0.52f, 0.16f);

                    MmUtil.mmcolor = Color.yellow;
                    var go1 = MmUtil.CreateCube(formgo, size: sphrad / 3);
                    go1.name = $"box";
                    // 7x5.4x4.3.5
                    go1.transform.position = new Vector3(0.0f, 0.0f, -0.16f);
                    go1.transform.localScale = new Vector3(0.56f, 0.32f, 0.28f);

                    MmUtil.mmcolor = MmUtil.GetRandomColor();
                    var go2 = MmUtil.CreateSphere(formgo, size: sphrad / 3);
                    go2.name = $"nose";
                    go2.transform.position = new Vector3(0.0f, 0.2f, -0.16f);
                    go2.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                    break;
                }
            case SledForm.Prefab:
                {
                    MmUtil.mmcolor = Color.gray;
                    var prefab = (GameObject)Resources.Load("Prefabs/Sled");
                    var go = Instantiate<GameObject>(prefab);
                    go.name = $"tray";
                    // 6.5x11.0x2cm
                    go.transform.parent = formgo.transform;
                    go.transform.position = new Vector3(0.0f, 0.0f, 0.088f);
                    go.transform.localRotation = Quaternion.Euler(180, 90, -90);
                    go.transform.localScale = new Vector3(8, 8, 8);
                    //go.transform.localScale = new Vector3(0.88f, 0.52f, 0.16f);

                    //MmUtil.mmcolor = Color.yellow;
                    var prefab1 = (GameObject) Resources.Load("Prefabs/Box1");
                    var go1 = Instantiate<GameObject>(prefab1);
                    go1.name = $"box";
                    // 7x5.4x4.3.5
                    go1.transform.parent = formgo.transform;
                    go1.transform.position = new Vector3(0.0f, 0.0f, -0.16f);
                    go1.transform.localRotation = Quaternion.Euler(180,90,-90);
                    go1.transform.localScale = new Vector3(8, 8, 8);


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
    }

    int updatecount = 0;
    // Update is called once per frame
    void Update()
    {
        updatecount++;
        if (speed>0 && pathnum>=0)
        {
            //Debug.Log($"Updating {sledid}");
            var deltdist = speed * Time.deltaTime;
            var path = mmt.GetPath(pathnum);
            bool markfordeletion;
            (pathnum, pathdist, markfordeletion) = path.AdvancePathdist(pathdist, deltdist);
            if (markfordeletion)
            {
                this.MarkForDeletion();
            }
            var newpath = mmt.GetPath(pathnum);
            var (pt, ang) = newpath.GetPositionAndOrientation(pathdist);
            var geomparenttrans = geomgo.transform.parent;
            geomgo.transform.parent = null;
            geomgo.transform.position = pt;
            geomgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
            geomgo.transform.SetParent(geomparenttrans, worldPositionStays:false);
        }
    }
}
