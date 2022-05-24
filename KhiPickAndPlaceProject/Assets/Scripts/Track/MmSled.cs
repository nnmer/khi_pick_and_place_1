using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KhiDemo
{

    public class MmSled : MonoBehaviour
    {
        MmTable mmt;
        public enum SledForm { BoxCubeBased, Prefab }
        static float sphrad = 0.2f;
        public int pathnum;
        public float pathdist;
        bool markedForDeletion = false;
        public float sledspeed;
        public bool visible;
        SledForm sledform;
        GameObject geomgo;
        GameObject formgo;
        GameObject boxgo;
        public bool loadState;
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
        public bool GetLoadState()
        {
            return loadState;
        }
        public void SetLoadState(bool newLoadState)
        {
            if (newLoadState == loadState) return;
            loadState = newLoadState;
            if (boxgo != null)
            {
                boxgo.SetActive(loadState);
            }
        }
        public void DeleteStuff()
        {
            var parentgo = formgo.transform.parent.gameObject;
            Destroy(geomgo);
        }

        public static MmSled ConstructSled(MmTable mmt, string sledid, int pathnum, float pathdist, bool loaded)
        {
            var sname1 = $"sledid:{sledid}";
            var sledgeomgo = new GameObject(sname1);
            var (pt, ang) = mmt.GetPositionAndOrientation(pathnum, pathdist);
            sledgeomgo.transform.position = pt;
            sledgeomgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
            var sled = sledgeomgo.AddComponent<MmSled>();
            var sledform = MmSled.SledForm.Prefab;
            sled.ConstructForm(mmt, sledgeomgo, sledform, sledid, pathnum, pathdist, loaded);
            sledgeomgo.transform.SetParent(mmt.mmtgo.transform, worldPositionStays: true);
            return sled;
            //Debug.Log($"makesled pathnum:{pathnum} dist:{pathdist:f1} pt:{sledgeomgo.transform.position:f1}");
        }

        public void ConstructForm(MmTable mmt, GameObject geomgo, SledForm sledform, string sledid, int pathnum, float pathdist, bool loaded)
        {
            this.geomgo = geomgo;
            this.mmt = mmt;
            this.sledform = sledform;
            this.sledspeed = 0;
            formgo = new GameObject("sledform");
            this.pathnum = pathnum;
            if (pathnum == 0)
            {
                pathnum = 0;
            }
            this.pathdist = pathdist;
            this.sledid = sledid;
            var (pt, ang) = mmt.GetPositionAndOrientation(pathnum, pathdist);
            switch (this.sledform)
            {
                case SledForm.BoxCubeBased:
                    {
                        var go = UnityUt.CreateCube(formgo, "gray", size: sphrad / 3);
                        go.name = $"tray";
                        // 6.5x11.0x2cm
                        go.transform.localScale = new Vector3(0.88f, 0.52f, 0.16f);

                        var go1 = UnityUt.CreateCube(formgo, "yellow", size: sphrad / 3);
                        go1.name = $"box";
                        // 7x5.4x4.3.5
                        go1.transform.position = new Vector3(0.0f, 0.0f, -0.16f);
                        go1.transform.localScale = new Vector3(0.56f, 0.32f, 0.28f);
                        boxgo = go1;

                        var clr = UnityUt.GetRandomColorString();
                        var go2 = UnityUt.CreateSphere(formgo, clr, size: sphrad / 3);
                        go2.name = $"nose";
                        go2.transform.position = new Vector3(0.0f, 0.2f, -0.16f);
                        go2.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                        break;
                    }
                case SledForm.Prefab:
                    {
                        var prefab = (GameObject)Resources.Load("Prefabs/Sled");
                        var go = Instantiate<GameObject>(prefab);
                        go.name = $"tray";
                        // 6.5x11.0x2cm
                        go.transform.parent = formgo.transform;
                        go.transform.position = new Vector3(0.0f, 0.0f, 0.088f);
                        go.transform.localRotation = Quaternion.Euler(180, 90, -90);
                        go.transform.localScale = new Vector3(8, 8, 8);
                        //go.transform.localScale = new Vector3(0.88f, 0.52f, 0.16f);

                        var prefab1 = (GameObject)Resources.Load("Prefabs/Box1");
                        var go1 = Instantiate<GameObject>(prefab1);
                        go1.name = $"box";
                        // 7x5.4x4.3.5
                        go1.transform.parent = formgo.transform;
                        go1.transform.position = new Vector3(0.0f, 0.0f, -0.16f);
                        go1.transform.localRotation = Quaternion.Euler(180, 90, -90);
                        go1.transform.localScale = new Vector3(8, 8, 8);
                        boxgo = go1;

                        var clr = UnityUt.GetRandomColorString();
                        var go2 = UnityUt.CreateSphere(formgo, clr, size: sphrad / 3);
                        go2.name = $"nose";
                        go2.transform.position = new Vector3(0.0f, 0.2f, -0.16f);
                        go2.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                        break;
                    }
            }
            loadState = loaded;
            boxgo.SetActive(loadState);
            visible = pathnum >= 0;
            formgo.SetActive(visible);
            formgo.transform.position = pt;
            formgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
            var rot1 = new Vector3(0, 90, -90);
            var rot2 = -rot1;
            var off1 = new Vector3(-0.27f, 0, -0.12f);
            var off2 = new Vector3(+0.27f, 0, -0.12f);
            var txt = $"{sledid}";
            var meth = UnityUt.FltTextImpl.TextPro;
            UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt, "yellow", rot1, off1, meth);
            UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt, "yellow", rot2, off2, meth);

            if (mmt.useMeters)
            {
                var u2m = mmt.UnitsToMeters;
                formgo.transform.localScale = new Vector3(u2m, u2m, u2m);
            }
            formgo.transform.SetParent(geomgo.transform, worldPositionStays: true);
            AdjustSledPositionAndOrientation(pt, ang);
            //Debug.Log($"ConstructSledForm pathnum:{pathnum} dist:{pathdist:f1} pt:{formgo.transform.position:f1}");
        }


        void AdjustSledPositionAndOrientation(Vector3 pt, float ang)
        {
            var geomparenttrans = geomgo.transform.parent;
            geomgo.transform.parent = null;
            geomgo.transform.position = pt;
            geomgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
            geomgo.transform.localScale = Vector3.one;
            geomgo.transform.SetParent(geomparenttrans, worldPositionStays: false);
            geomgo.transform.SetAsFirstSibling();
        }

        int last_pathnum;
        float last_pathdist;
        bool last_loaded;
        float last_time;
        static float max_speed = 0;
        static float avg_speed = 0;
        static float sum_speed = 0;
        static int nspeed_calcs = 0;


        public void UpdateSled(int new_pathnum, float new_pathdist, bool new_loaded)
        {
            var msg = $"Updating {sledid} to path:{new_pathnum} pos:{new_pathdist:f2} loaded:{new_loaded}";
            Debug.Log(msg);
            this.pathnum = new_pathnum;
            this.pathdist = new_pathdist;
            this.visible = new_pathnum >= 0;
            this.formgo.SetActive(visible);
            if (new_pathnum < 0) return;

            var (pt, ang) = mmt.GetPositionAndOrientation(new_pathnum, new_pathdist);
            AdjustSledPositionAndOrientation(pt, ang);
            SetLoadState(new_loaded);
            if (last_pathnum == new_pathnum)
            {
                var deltatime = Time.time - last_time;
                if (mmt.interpolateOnSpeed && (deltatime > 0))
                {
                    sledspeed = (new_pathdist - last_pathdist) / deltatime;
                    if (sledspeed < 0)
                    {
                        sledspeed = 0;
                    }
                    nspeed_calcs++;
                    sum_speed += sledspeed;
                    avg_speed = sum_speed / nspeed_calcs;
                    if (sledspeed > max_speed)
                    {
                        max_speed = sledspeed;
                    }
                    Debug.Log($"sled {this.sledid} sledspeed:{sledspeed:f3}  avg_speed:{avg_speed:f3} sum_speed:{sum_speed}   max_speed:{max_speed:f3} nspeed_calcs:{nspeed_calcs}");
                }
            }
            last_pathnum = new_pathnum;
            last_pathdist = new_pathdist;
            last_loaded = new_loaded;
            last_time = Time.time;
        }
        public void AdvanceSledBySpeed()
        {
            if (pathnum >= 0)
            {
                var deltdist = this.sledspeed * Time.deltaTime;
                var path = mmt.GetPath(pathnum);
                bool markfordeletion;
                (pathnum, pathdist, markfordeletion) = path.AdvancePathdist(pathdist, deltdist);
                if (markfordeletion)
                {
                    this.MarkForDeletion();
                }
                var newpath = mmt.GetPath(pathnum);
                var (pt, ang) = newpath.GetPositionAndOrientation(pathdist);
                AdjustSledPositionAndOrientation(pt, ang);
            }
        }
        int updatecount = 0;
        // Update is called once per frame

        void Update()
        {
            updatecount++;
        }
    }
}