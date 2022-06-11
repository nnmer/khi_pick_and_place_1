using System.Collections.Generic;
using UnityEngine;
using MmSledMsg = RosMessageTypes.Rs007Control.MagneMotionSledMsg;
using Unity.Robotics.ROSTCPConnector;

namespace KhiDemo
{
    public enum SledSpeedDistrib { fixedValue, alternateHiLo }
    public enum SledLoadDistrib { allLoaded, allUnloaded, alternateLoadedUnloaded }

    public class MmTable : MonoBehaviour
    {
        public MagneMotion magmo;

        public GameObject mmtgo;
        public GameObject pago;
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
            magmo.ros.Subscribe<MmSledMsg>("Rs007Sleds", EchoSledChange);
            magmo.ros.RegisterPublisher<MmSledMsg>("Rs007Sleds");
        }

        public void PublishSleds()
        {
            if (magmo.publishMovements)
            {
                //Debug.Log("PublishSleds");
                foreach (var s in sleds)
                {
                    //Debug.Log($"   sled:{s.sledidx}");
                    var sledmsg = new MmSledMsg(s.loadState, s.pathUnitDist, s.pathnum, s.sledidx);
                    magmo.ros.Publish("Rs007Sleds", sledmsg);
                }
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
        public void MakeMsftDemoMagmo()
        {
            //Debug.Log("Making MsftDemoMagmo");
            tableName = "MsftDemoMagmo";
            var mmt = this;
            var ptstar = new Vector3(4, 0, 0);
            var p1 = mmt.MakePath("path1", ptstar);
            p1.MakeLineSeg("w", 2);
            p1.MakeLineSeg("w", 8);
            p1.MakeLineSeg("w", 2);
            p1.MakeCircSeg("s", "cw");
            p1.MakeCircSeg("w", "cw");

            var p2 = mmt.MakePath("path2", p1.End());
            p2.MakeCircSeg("s", "ccw");
            p1.LinkToContinuationPath(p2);
            p1.SetPreferedLoadedPath(p2);

            var p3 = mmt.MakePath("path3", p2.End());
            p3.MakeLineSeg("n", 2);
            //p2.LinkToContinuationPath(p3);

            var p4 = mmt.MakePath("path4", p2.End());
            p4.MakeCircSeg("w", "cw");
            p4.MakeLineSeg("e", 2);
            p4.MakeLineSeg("e", 8);
            p4.MakeLineSeg("e", 2);
            p4.MakeCircSeg("n", "cw");
            p4.MakeCircSeg("w", "ccw");
            p2.LinkToContinuationPath(p4);
            p4.SetLoadedStopPoint(5.0f);

            var p5 = mmt.MakePath("path5", p1.End());
            p5.MakeLineSeg("e", 2);
            p5.MakeLineSeg("e", 8);
            p5.MakeLineSeg("e", 2);
            p1.LinkToContinuationPath(p5);
            p1.SetPreferedUnloadedPath(p5);
            p5.SetUnloadedStopPoint(6.0f);

            var p6 = mmt.MakePath("path6", p5.End());
            p6.MakeLineSeg("e", 2);
            p6.MakeLineSeg("e", 2);
            p5.LinkToContinuationPath(p6);

            var p7 = mmt.MakePath("path7", p4.End());
            p7.MakeCircSeg("n", "cw");
            p7.MakeCircSeg("e", "cw");
            p7.MakeLineSeg("w", 2);
            p7.MakeLineSeg("w", 2);
            p4.LinkToContinuationPath(p7);
            p6.LinkToContinuationPath(p7);


            var p8 = mmt.MakePath("path8", p7.End());
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
            var p1 = mmt.MakePath("path1", ptstar);
            p1.MakeLineSeg("n", 8);

            var p2 = mmt.MakePath("path2", p1.End());
            p2.MakeCircSeg("w", "cw");
            p2.MakeLineSeg("e", 4);
            p2.MakeCircSeg("n", "cw");
            p2.MakeLineSeg("s", 8);
            p1.LinkToContinuationPath(p2);

            var p3 = mmt.MakePath("path3", p2.End());
            p3.MakeCircSeg("e", "cw");
            p3.MakeLineSeg("w", 4);
            p3.MakeCircSeg("s", "cw");
            p2.LinkToContinuationPath(p3);
            p3.LinkToContinuationPath(p1);
        }

        SledSpeedDistrib sledSpeedDistribution; 

        public void SetupSleds(SledLoadDistrib sledLoading, SledSpeedDistrib sledSpeedDist, float sledspeed)
        {
            this.sledSpeedDistribution = sledSpeedDist;
            switch (sledSpeedDist)
            {
                case SledSpeedDistrib.fixedValue:
                    foreach (var s in sleds)
                    {
                        s.SetSpeed(sledspeed);
                    }
                    break;
                case SledSpeedDistrib.alternateHiLo:
                    int i = 0;
                    foreach (var s in sleds)
                    {
                        float val = (i % 2 == 0) ? sledspeed : sledspeed/2;
                        //Debug.Log($"Set {s.sledid} speed to {val}");
                        s.SetSpeed(val);
                        i++;
                    }
                    break;
            }
            switch (sledLoading)
            {
                case SledLoadDistrib.allLoaded:
                    foreach (var s in sleds)
                    {
                        s.SetLoadState(true);
                    }
                    break;
                case SledLoadDistrib.allUnloaded:
                    foreach (var s in sleds)
                    {
                        s.SetLoadState(false);
                    }
                    break;
                case SledLoadDistrib.alternateLoadedUnloaded:
                    var i = 0;
                    foreach (var s in sleds)
                    {
                        bool val = (i % 2 == 0) ? true : false;
                        s.SetLoadState(val);
                        i++;
                    }
                    break;
            }
            foreach(var s in sleds)
            {
                s.SetNextPath();
            }
        }

        public void AdjustSledSpeedFactor(float fak)
        {
            foreach (var s in sleds)
            {
                var speed = s.sledUpsSpeed;
                s.SetSpeed(speed*fak);
            }
        }


        MmPath MakePath(string name, Vector3 pt)
        {
            var idx = paths.Count;
            var rv = new MmPath(magmo, idx, name, pt);
            paths.Add(rv);
            return rv;
        }
        public GameObject SetupGeometry(bool addPathMarkers, bool positionOnFloor)
        {
            mmtgo = new GameObject(tableName);
            pago = new GameObject("pago");
            pago.transform.SetParent(mmtgo.transform, worldPositionStays: false);

            if (addPathMarkers)
            {
                foreach (var p in paths)
                {
                    p.AddPathRails(pago, seggos: false, pathgos: true);
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

        public MmSled MakeSled(int sledidx, string sledid, int pathnum, float pathdist, bool loaded)
        {
            var sled = MmSled.ConstructSled(magmo, sledidx, sledid, pathnum, pathdist, loaded);
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
                        if (totsleds < 10) // only create 10
                        {
                            var frac = i * 1.0f / nsleds;
                            var pathdist = frac * p.pathLength;
                            var iid = sleds.Count + 1;
                            var sledid = $"{iid}";
                            var loaded = (i % 2 == 0);
                            var sled = MakeSled(iid,sledid, p.pidx, pathdist, loaded: loaded);
                            totsleds++;
                        }
                    }
                }
            }
        }

        public (int nloadedstopped, int nunloadedstopped) CountStoppedSleds()
        {
            var nloadedstopped = 0;
            var nunloadedstopped = 0;
            foreach (var s in sleds)
            {
                if (s.stopped)
                {
                    if (s.loadState)
                    {
                        nloadedstopped++;
                    }
                    else
                    {
                        nunloadedstopped++;
                    }
                }
            }
            return (nloadedstopped, nunloadedstopped);
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
            if (magmo.mmMode== MmMode.Echo)
            {
                // don't advance sled
                return;
            }
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
            if (magmo.echoMovements)
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
                    //if (oldstate != loaded)
                    //{
                    //    Debug.Log($"Sled {sledid} changed loaded state to {loaded}");
                    //}
                }
                else
                {
                    //Debug.Log($"making sled:{sledid}");
                    if (pathid >= 0)
                    {
                        MakeSled(sledmsg.cartid, sledid, pathid, position, loaded);
                    }
                }
            }
        }

        public void PhysicsStep()
        {
            DeleteSledsAsNeeded();
            AdvanceSledsBySpeed();
        }
    }
}