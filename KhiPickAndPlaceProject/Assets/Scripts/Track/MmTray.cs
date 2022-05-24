using System.Collections.Generic;
using UnityEngine;
using MmTray1Msg = RosMessageTypes.Rs007Control.MagneMotionTray1Msg;
using Unity.Robotics.ROSTCPConnector;

namespace KhiDemo
{

    public class MmTray : MonoBehaviour
    {
        MmTable mmt;

        public enum TrayBoxForm { Box, Prefab }

        public GameObject mmtrayrep;
        public GameObject mmtraygo;

        const int nrow = 3;
        const int ncol = 4;
        Dictionary<(int, int), bool> loaded = new Dictionary<(int, int), bool>();
        Dictionary<(int, int), GameObject> box = new Dictionary<(int, int), GameObject>();

        bool positionOnFloor = true;
        TrayBoxForm trayBoxForm = TrayBoxForm.Prefab;

        public MmTray()
        {
            initVals();
        }

        (bool ok, string errmsg) CheckIndexes(int i, int j, string rooname)
        {
            if (i < 0 || nrow <= i)
            {
                var msg = $"row out of rannge:{i} max:{nrow - 1} in {rooname}";
                return (false, msg);
            }
            if (j < 0 || ncol <= j)
            {
                var msg = $"col out of rannge:{j} max:{ncol - 1} in {rooname}";
                return (false, msg);
            }
            return (true, "");
        }


        void Start()
        {
            ROSConnection.GetOrCreateInstance().Subscribe<MmTray1Msg>("Rs007Tray1", Tray1Change);
        }

        public void Init(MmTable mmt)
        {
            this.mmt = mmt;
            // size in cm - 31.4, 28.9, 1.7
            CreateTray(gameObject.transform);
            CreateBoxes();
        }
        void CreateTray(Transform parent)
        {
            mmtrayrep = new GameObject("mmtrayrep");

            //// move it behind the robot and up to the first robot joint 
            //mmtrayrep.transform.position = new Vector3(0.376f, 0.256f, 0.0324f);
            //mmtrayrep.transform.localRotation = Quaternion.Euler(-90.0f, 0.0f, 0.0f);

            mmtrayrep.transform.SetParent(parent, worldPositionStays: false);
            //mmtrayrep.transform.position = new Vector3(0.182f, -0.06199f, 2.6299f);

            // attach to floor if there is one
            mmtraygo = UnityUt.CreateCube(mmtrayrep, "gray", size: 1, wps: false);
            mmtraygo.name = "mmtraygo";
            mmtraygo.transform.localScale = new Vector3(0.289f, 0.017f, 0.314f);
        }



        void CreateBoxes()
        {
            var rowdelt = 0.03f * 3;
            var coldelt = 0.022f * 3;
            var rowstar = rowdelt * (((float)nrow - 1) / 2f);
            var colmid = 0.003f;
            var colstar = coldelt * (((float)ncol - 1) / 2f);
            var rowpos = rowstar;
            for (var i = 0; i < nrow; i++)
            {
                var colpos = colstar;
                for (var j = 0; j < ncol; j++)
                {
                    // so the columns are not evenly distributed (sigh)
                    var colpos1 = colpos - colmid;
                    if (j < (ncol / 2))
                    {
                        colpos1 = colpos + colmid;
                    }
                    var pt = new Vector3(colpos1, 0.02f, rowpos);
                    GameObject boxgo = null;

                    switch (trayBoxForm)
                    {
                        case TrayBoxForm.Box:
                            {
                                boxgo = UnityUt.CreateCube(null, "yellow", size: 1);
                                boxgo.transform.localScale = new Vector3(0.04f, 0.04f, 0.06f);
                                break;
                            }
                        case TrayBoxForm.Prefab:
                            {
                                var prefab = Resources.Load<GameObject>("Prefabs/Box1");
                                boxgo = Instantiate<GameObject>(prefab);
                                var ska = 1f;
                                boxgo.transform.localScale = new Vector3(ska, ska, ska);
                                boxgo.transform.localRotation = Quaternion.Euler(0, 90, 0);
                                break;
                            }
                    }
                    boxgo.name = $"box {i}-{j}";
                    boxgo.transform.parent = null;
                    boxgo.transform.position = pt;
                    boxgo.transform.SetParent(mmtrayrep.transform, worldPositionStays: false);
                    box[(i, j)] = boxgo;
                    loaded[(i, j)] = true;
                    colpos -= coldelt;
                }
                rowpos -= rowdelt;
            }
            RealizeLoadStatus();
        }

        void RealizeLoadStatus()
        {
            for (var i = 0; i < nrow; i++)
            {
                for (var j = 0; j < ncol; j++)
                {
                    var bkey = (i, j);
                    box[bkey].SetActive(loaded[bkey]);
                }
            }
        }

        void Tray1Change(MmTray1Msg traymsg)
        {
            //Debug.Log($"Received ROS message on topic Rs007Tray1:{traymsg.ToString()}");
            var (ok, msg) = CheckIndexes(traymsg.row, traymsg.col, "Tray1Change");
            if (ok)
            {
                var oldstat = GetVal(traymsg.row, traymsg.col);
                var newstat = traymsg.loaded != 0;
                //Debug.Log($"   oldstat:{oldstat} newstat:{newstat}");
                if (oldstat != newstat)
                {
                    SetVal(traymsg.row, traymsg.col, traymsg.loaded != 0);
                    RealizeLoadStatus();
                }
            }
            else
            {
                Debug.LogError(msg);
            }
        }

        bool GetVal(int i, int j)
        {
            var key = (i, j);
            if (!loaded.ContainsKey(key)) return false;
            var rv = loaded[key];
            return rv;
        }

        void SetVal(int i, int j, bool newstat)
        {
            var key = (i, j);
            loaded[key] = newstat;
        }
        void initVals()
        {
            for (int i = 1; i < nrow; i++)
            {
                for (int j = 1; j < ncol; j++)
                {
                    loaded[(i, j)] = false;
                }
            }
        }


        // Update is called once per frame
        void Update()
        {
        }
    }
}