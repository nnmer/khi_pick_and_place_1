using System.Collections.Generic;
using UnityEngine;
using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
using Unity.Robotics.ROSTCPConnector;

namespace KhiDemo
{
    public enum SledSpeedDistribution { fixedValue, alternateHiLo }

    public class MnTable : MonoBehaviour
    {
        public MagneMotion magmo;

        public MmBoxMode mmBoxMode = MmBoxMode.Fake;
        public GameObject mmtgo;
        public string tableName = "TableName";
        public List<MmPath> paths = new List<MmPath>();
        public float UnitsToMeters = 0.125f;
        public GameObject sledsgo;
        public List<MmSled> sleds = new List<MmSled>();
        public List<MmRail> rails = new List<MmRail>();

        public bool useMeters = true;
        public float sledSpeed = 0.01f;
        public bool interpolateOnSpeed = false;



        public Dictionary<string, MmSled> SledDict = new Dictionary<string, MmSled>();

        // Start is called before the first frame update
        void Start()
        {
            ROSConnection.GetOrCreateInstance().Subscribe<MmSledMsg>("Rs007Sleds", EchoSledChange);
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

        public void Init(MagneMotion magmo)
        {
            this.magmo = magmo;

        }

        public (Vector3 pt, float ang) GetPositionAndOrientation(int pathnum, float pathdist)
        {
            var path = GetPath(pathnum);
            var (pt, ang) = path.GetPositionAndOrientation(pathdist);
            return (pt, ang);
        }

        public void MakeMsftDemoMagmo(MmMode mode)
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
            p1.SetPreferedLoadedPath(p2);

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
            p4.SetLoadedStopPoint(5.0f);

            var p5 = mmt.makePath("path5", p1.End());
            p5.MakeLineSeg("e", 2);
            p5.MakeLineSeg("e", 8);
            p5.MakeLineSeg("e", 2);
            p1.LinkToContinuationPath(p5);
            p1.SetPreferedUnloadedPath(p5);
            if (mode == MmMode.SimulateRailToTray)
            {
                p5.SetUnloadedStopPoint(6.0f);
            }

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
            //p7.LinkToContinuationPath(p8);
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
            p2.MakeLineSeg("e", 4);
            p2.MakeCircSeg("n", "cw");
            p2.MakeLineSeg("s", 8);
            p1.LinkToContinuationPath(p2);

            var p3 = mmt.makePath("path3", p2.End());
            p3.MakeCircSeg("e", "cw");
            p3.MakeLineSeg("w", 4);
            p3.MakeCircSeg("s", "cw");
            p2.LinkToContinuationPath(p3);
            p3.LinkToContinuationPath(p1);
        }

        SledSpeedDistribution sledSpeedDistribution; 

        public void SetSledUpsSpeed(SledSpeedDistribution sledSpeedDistribution, float sledspeed)
        {
            this.sledSpeedDistribution = sledSpeedDistribution;
            switch (sledSpeedDistribution)
            {
                case SledSpeedDistribution.fixedValue:
                    foreach (var s in sleds)
                    {
                        s.SetSpeed(sledspeed);
                    }
                    break;
                case SledSpeedDistribution.alternateHiLo:
                    int i = 0;
                    foreach (var s in sleds)
                    {
                        float val = (i % 2 == 0) ? sledspeed : sledspeed/2;
                        Debug.Log($"Set {s.sledid} speed to {val}");
                        s.SetSpeed(val);
                        i++;
                    }
                    break;
            }
        }


        MmPath makePath(string name, Vector3 pt)
        {
            var idx = paths.Count;
            var rv = new MmPath(magmo, idx, name, pt);
            paths.Add(rv);
            return rv;
        }
        public GameObject SetupGeometry(bool addPathMarkers, bool positionOnFloor)
        {
            mmtgo = new GameObject(tableName);

            if (addPathMarkers)
            {
                foreach (var p in paths)
                {
                    p.AddPathRails(mmtgo, seggos: false, pathgos: true);
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

        public MmSled MakeSled(string sledid, int pathnum, float pathdist, bool loaded)
        {
            var sled = MmSled.ConstructSled(magmo, sledid, pathnum, pathdist, loaded);
            this.sleds.Add(sled);
            this.SledDict[sledid] = sled;
            return sled;
        }
        public void AddSleds()
        {
            // Add Sleds so the begining scene is not quite so empty
            var totsleds = 0;
            foreach (var p in paths)
            {
                if (p.continuationPaths.Count > 0) // no dead edns
                {
                    var nsleds = (int)p.pathLength / 3.0f;
                    for (int i = 0; i < nsleds; i++)
                    {
                        if (totsleds <= 10) // only create 10
                        {
                            var frac = i * 1.0f / nsleds;
                            var pathdist = frac * p.pathLength;
                            var iid = sleds.Count + 1;
                            var sledid = $"{iid}";
                            var loaded = (i % 2 == 0);
                            var sled = MakeSled(sledid, p.pidx, pathdist, loaded: loaded);
                            totsleds++;
                        }
                    }
                }
            }
        }

        public MmSled FindStoppedSled(bool neededLoadState)
        {
            foreach(var s in sleds)
            {
                if (s.stopped && neededLoadState==s.loadState)
                {
                    return s;
                }
            }
            return null;
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

        public void AdvanceSledsBySpeed()
        {
            foreach (var sled in sleds)
            {
                sled.FindSledInFront();
            }
            foreach (var sled in sleds)
            {
                sled.AdvanceSledBySpeed();
            }
        }


        void EchoSledChange(MmSledMsg sledmsg)
        {
            // Debug.Log($"Received ROS message on topic Rs007Sleds:{sledmsg.ToString()}");
            var sledid = $"{sledmsg.cartid}";
            var loaded = sledmsg.loaded;
            var pathid = sledmsg.pathid - 1;
            var position = (float)sledmsg.position / UnitsToMeters;
            if (sledmsg.pathid < 0)
            {
                Debug.LogWarning($"Bad pathid detected {sledmsg.pathid} on cartid:{sledmsg.cartid}");
                return;
            }
            if (SledDict.ContainsKey(sledid))
            {
                var sled = SledDict[sledid];
                var oldstate = sled.GetLoadState();
                sled.EchoUpdateSled(pathid, position, loaded);
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
                    MakeSled(sledid, pathid, position, loaded);
                }
            }
        }



        int updatecount = 0;
        // Update is called once per frame
        void Update()
        {
            DeleteSledsAsNeeded();
            AdvanceSledsBySpeed();
            updatecount++;
        }
    }
}