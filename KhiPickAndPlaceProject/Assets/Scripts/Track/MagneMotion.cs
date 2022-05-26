using System.Collections.Generic;
using UnityEngine;
using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
using Unity.Robotics.ROSTCPConnector;

namespace KhiDemo
{

    public enum MmBoxMode { Fake, Real }

    public enum MmSegForm { None, Straight, Curved }

    public class MmTable
    {
        public GameObject robmodel;
        public GameObject vgrip;
        public MmBoxMode mmBoxMode = MmBoxMode.Fake;
        public MmSled.SledForm mmSledForm = MmSled.SledForm.Prefab;
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


        public MmSled MakeSled(string sledid, int pathnum, float pathdist, bool loaded)
        {
            var sled = MmSled.ConstructSled(this, sledid, pathnum, pathdist, loaded);
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
        GameObject boxgo;
        public void AddBoxToRobot(Transform vgriptrans)
        {
            var prefab = Resources.Load<GameObject>("Prefabs/Box1");
            boxgo = Object.Instantiate<GameObject>(prefab);
            boxgo.name = "RobBox";
            var ska = 1f;
            boxgo.transform.localScale = new Vector3(ska, ska, ska);
            boxgo.transform.localRotation = Quaternion.Euler(0, 0, 0);
            boxgo.transform.localPosition = new Vector3(0, -0.14f, 0);
            boxgo.transform.SetParent(vgriptrans, worldPositionStays: false);
            ActivateRobBox(false);
        }
        float lasttimeset = -99;
        float lockpause = 0.01f;
        public bool ActivateRobBox(bool newstat)
        {
            var rv = false;
            if (boxgo != null)
            {
                if ((Time.time - lasttimeset) > lockpause)
                {
                    rv = boxgo.activeSelf;
                    boxgo.SetActive(newstat);
                    lasttimeset = Time.time;
                }
            }
            return rv;
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
        public GameObject robmodel;
        public MmTable mmtable = null;
        public MmTray mmtray = null;
        public Rs007TrajectoryPlanner planner = null;
        public MmSled.SledForm sledForm = MmSled.SledForm.Prefab;

        // Start is called before the first frame update
        void Start()
        {
            planner = GameObject.FindObjectOfType<Rs007TrajectoryPlanner>();
            mmtable = new MmTable();
            mmtable.useMeters = useMeters;
            mmtable.mmSledForm = sledForm;
            mmtable.MakeMsftDemoMagmo();
            if (robmodel == null)
            {
                var urdf = FindObjectOfType<Unity.Robotics.UrdfImporter.UrdfRobot>();
                if (robmodel)
                {
                    Debug.LogError("Robmodel not set in Magnemotion table");
                }
                else
                {
                    robmodel = urdf.gameObject;
                }
            }
            else
            {
                mmtable.robmodel = robmodel;
                mmtable.AddBoxToRobot(planner.vgriptrans);
            }

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
        int updatecount = 0;
        MmSled.SledForm oldsledForm;
        // Update is called once per frame
        void Update()
        {
            if (updatecount++ == 0)
            {
                oldsledForm = sledForm;
            }
            if (sledForm != oldsledForm)
            {
                foreach (var sled in mmtable.sleds)
                {
                    sled.ConstructForm(sledForm);
                }
                mmtable.mmSledForm = sledForm;
                oldsledForm = sledForm;
            }
            mmtable.DeleteSledsAsNeeded();
            mmtable.UpdateTable();
        }
    }
}