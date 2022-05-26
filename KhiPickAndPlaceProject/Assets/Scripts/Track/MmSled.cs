using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace KhiDemo
{

    public class MmSled : MonoBehaviour
    {
        MagneMotion mmt;
        public enum SledForm { BoxCubeBased, Prefab, PrefabWithBox }
        public int pathnum;
        public float pathdist;
        bool markedForDeletion = false;
        public float sledspeed;
        public bool visible;
        SledForm sledform;
        GameObject formgo;
        GameObject boxgo;
        public bool loadState;
        public string sledid;

        public static MmSled ConstructSled(MagneMotion mmt, string sledid, int pathnum, float pathdist, bool loaded)
        {
            var sname1 = $"sledid:{sledid}";
            var sledgo = new GameObject(sname1);
            //var (pt, ang) = mmt.GetPositionAndOrientation(pathnum, pathdist);
            //sledgo.transform.position = pt;
            //sledgo.transform.rotation = Quaternion.Euler(0, 0, -ang);
            var sledform = mmt.sledForm;

            var sled = sledgo.AddComponent<MmSled>();
            sled.sledid = sledid;
            sled.mmt = mmt;
            // Set default state
            sled.sledspeed = 0;
            sled.pathnum = 0;
            sled.pathdist = 0;
            sled.loadState = true;
            sled.visible = true;

            sled.ConstructForm( sledform );
            sled.AdjustSledOnPathDist(pathnum, pathdist);
            sled.SetLoadState(loaded);
            sledgo.transform.SetParent(mmt.mmtgo.transform, worldPositionStays: true);
            return sled;
            //Debug.Log($"ConstructSled pathnum:{pathnum} dist:{pathdist:f1} pt:{sledgeomgo.transform.position:f1}");
        }

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
                mmt.ActivateRobBox(!loadState);
            }
        }
        public void DeleteStuff()
        {
            var parentgo = formgo.transform.parent.gameObject;
            Destroy(gameObject);
        }

        public void ConstructForm( SledForm sledform)
        {
            // This should have no parameters with changeable state except for the form
            // This ensures we can update the form without disturbing the other logic and state that the sled has, like position and loadstate
            // Coming out of this the 

            if (formgo!=null)
            {
                Destroy(formgo);
                formgo = null;
            }

            this.sledform = sledform;

            formgo = new GameObject("sledform");
            var ska8 = 1f/8;

            switch (this.sledform)
            {
                case SledForm.BoxCubeBased:
                    {
                        //var go = UnityUt.CreateCube(formgo, "gray", size: sphrad / 3);
                        //go.transform.localScale = new Vector3(0.88f, 0.52f, 0.16f);
                        // 6.5x11.0x2cm
                        var go = UnityUt.CreateCube(formgo, "gray", size: 1 );
                        go.transform.position = new Vector3(0.0f, 0.0f, 0.09f) * ska8;
                        go.transform.localScale = new Vector3(0.9f, 0.53f, 0.224f) * ska8;
                        go.name = $"tray";

                        var gobx = UnityUt.CreateCube(formgo, "yellow", size: 1);
                        gobx.name = $"box";
                        // 7x5.4x4.3.5
                        gobx.transform.position = new Vector3(0.0f, 0.0f, -0.16f) * ska8;
                        gobx.transform.localScale = new Vector3(0.43f, 0.56f, 0.27f) * ska8;
                        boxgo = gobx;

                        break;
                    }
                case SledForm.Prefab:
                    {
                        var prefab = (GameObject)Resources.Load("Prefabs/Sled");
                        var go = Instantiate<GameObject>(prefab);
                        go.name = $"tray";
                        // 6.5x11.0x2cm
                        go.transform.parent = formgo.transform;
                        go.transform.position = new Vector3(0.0f, 0.0f, 0.088f) * ska8;
                        go.transform.localRotation = Quaternion.Euler(180, 90, -90);
                        //go.transform.localScale = new Vector3(8, 8, 8);

                        var prefab1 = (GameObject)Resources.Load("Prefabs/Box1");
                        var gobx = Instantiate<GameObject>(prefab1);
                        gobx.name = $"box";
                        // 7x5.4x4.3.5
                        gobx.transform.parent = formgo.transform;
                        gobx.transform.position = new Vector3(0.0f, 0.0f, -0.16f)*ska8;
                        gobx.transform.localRotation = Quaternion.Euler(180, 90, -90);
                        //gobx.transform.localScale = new Vector3(8, 8, 8);
                        boxgo = gobx;

                        break;
                    }
                case SledForm.PrefabWithBox:
                    {
                        var prefab = (GameObject)Resources.Load("Prefabs/Sled");
                        var go = Instantiate<GameObject>(prefab);
                        go.name = $"tray";
                        // 6.5x11.0x2cm
                        go.transform.parent = formgo.transform;
                        go.transform.position = new Vector3(0.0f, 0.0f, 0.088f) * ska8;
                        go.transform.localRotation = Quaternion.Euler(180, 90, -90);
                        //go.transform.localScale = new Vector3(8, 8, 8);

                        var gobx = MmBox.ConstructBox(mmt, sledid);
                        gobx.name = "NewBox";
                        gobx.transform.parent = formgo.transform;
                        //gobx.transform.localScale = new Vector3(8, 8, 8);
                        boxgo = gobx.gameObject;

                        break;
                    }
            }

            AddSledIdToSledForm();

            //if (mmt.useMeters)
            //{
            //    var u2m = mmt.UnitsToMeters;
            //    formgo.transform.localScale = new Vector3(u2m, u2m, u2m);
            //}
            formgo.transform.SetParent(transform, worldPositionStays: false);
            Debug.Log($"ConstructSledForm sledForm:{sledform} id:{sledid}");
        }

        void AddSledIdToSledForm()
        {
            var ska = 1f/8;
            var rot1 = new Vector3(0, 90, -90);
            var rot2 = -rot1;
            var off1 = new Vector3(-0.27f, 0, -0.12f)*ska;
            var off2 = new Vector3(+0.27f, 0, -0.12f)*ska;
            var txt = $"{sledid}";
            var meth = UnityUt.FltTextImpl.TextPro;
            UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt, "yellow", rot1, off1, ska, meth);
            UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt, "yellow", rot2, off2, ska, meth);
        }

        void AdjustSledOnPathDist(int pathnum, float pathdist)
        {
            this.pathnum = pathnum;
            this.pathdist = pathdist;


            var (pt, ang) = mmt.GetPositionAndOrientation(pathnum, pathdist);
            AdjustSledPositionAndOrientation(pt, ang);

            visible = pathnum >= 0;
            formgo.SetActive(visible);
        }

        void AdjustSledPositionAndOrientation(Vector3 pt, float ang)
        {
            var geomparenttrans = transform.parent;
            transform.parent = null;

            transform.position = pt;
            transform.rotation = Quaternion.Euler(0, 0, -ang);
            transform.localScale = Vector3.one;
            transform.SetParent(geomparenttrans, worldPositionStays: false);
            transform.SetAsFirstSibling();
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

            AdjustSledOnPathDist(new_pathnum, new_pathdist);
            //var (pt, ang) = mmt.GetPositionAndOrientation(new_pathnum, new_pathdist);
            //AdjustSledPositionAndOrientation(pt, ang);
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
                AdjustSledOnPathDist(pathnum, pathdist);
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