using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace KhiDemo
{
    public enum BoxStatus { free, onTray, onSled, onRobot }
    public class MmBox : MonoBehaviour
    {
        MnTable mmt;
        GameObject formgo;
        public enum BoxForm { CubeBased, Prefab }
        BoxForm boxform;
        public string boxid1;
        public string boxid2;
        public int seqnum;
        public BoxStatus boxStatus;


        static int clas_seqnum = 0;


        public static MmBox ConstructBox(MagneMotion magmo, string boxid1, BoxStatus stat=BoxStatus.free)
        {
            var mmt = magmo.mmt;
            clas_seqnum++;
            var boxname = $"Box:{clas_seqnum}";
            var boxgeomgo = new GameObject(boxname);
            boxgeomgo.transform.position = Vector3.zero;
            boxgeomgo.transform.rotation = Quaternion.identity;
            var box = boxgeomgo.AddComponent<MmBox>();
            var boxform = magmo.boxForm;
            box.mmt = mmt;
            box.boxid1 = boxid1;

            box.seqnum = clas_seqnum;
            box.boxid2 = $"{box.seqnum}";
            box.ConstructForm(boxform);
            box.boxStatus = stat;
            //boxgeomgo.transform.SetParent(mmt.mmtgo.transform, worldPositionStays: true);
            return box;
            //Debug.Log($"makesled pathnum:{pathnum} dist:{pathdist:f1} pt:{sledgeomgo.transform.position:f1}");
        }
        // Start is called before the first frame update
        void Start()
        {

        }

        public void ConstructForm(BoxForm boxform)
        {

            if (formgo != null)
            {
                Destroy(formgo);
                formgo = null;
            }


            this.boxform = boxform;
            formgo = new GameObject("boxform");
            switch (this.boxform)
            {
                case BoxForm.CubeBased:
                    {
                        var gobx = UnityUt.CreateCube(formgo, "yellow", size: 1);
                        gobx.name = $"box";
                        // 7x5.4x4.3.5
                        gobx.transform.position = new Vector3(0.0f, 0.0f, -0.16f)*1f/8;
                        gobx.transform.localScale = new Vector3(0.43f, 0.56f, 0.26f)*1f/8;
                        break;
                    }
                case BoxForm.Prefab:
                    {
                        var prefab1 = (GameObject)Resources.Load("Prefabs/Box1");
                        var go1 = Instantiate<GameObject>(prefab1);
                        go1.name = $"box";
                        // 7x5.4x4.3.5
                        go1.transform.parent = formgo.transform;
                        go1.transform.position = new Vector3(0.0f, 0.0f, -0.16f)*1f/8;
                        go1.transform.localRotation = Quaternion.Euler(180, 90, -90);
                        //go1.transform.localScale = new Vector3(8, 8, 8);

                        break;
                    }
            }

            AddBoxIdToBoxForm();

            formgo.transform.SetParent(transform, worldPositionStays: false);
        }


        void AddBoxIdToBoxFormOld()
        {
            if (boxid1 != "")
            {
                var ska = 1f/8;
                var rot1 = new Vector3(0, 90, -90);
                var rot2 = -rot1;
                var off1 = new Vector3(-0.22f, -0.23f, -0.25f);
                var off2 = new Vector3(+0.22f, +0.23f, -0.25f);
                var txt = $"{boxid1}";
                var meth = UnityUt.FltTextImpl.TextPro;
                UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt, "black", rot1, off1, ska, meth);
                UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt, "black", rot2, off2, ska, meth);
            }
        }
        void AddBoxIdToBoxForm()
        {
            if (boxid1 != "")
            {
                var rot1 = new Vector3(0, 90, -90);
                var rot2 = -rot1;
                var rot3 = new Vector3(0, 0, 0);
                var off1 = new Vector3(-0.0275f, -0.02875f, -0.03125f);
                var off2 = new Vector3(+0.0275f, +0.02875f, -0.03125f);
                var off3 = new Vector3(+0.0188f, +0.02875f, -0.0366f);
                var txt1 = $"{boxid1}";
                var txt2 = $"{boxid2}";
                var meth = UnityUt.FltTextImpl.TextPro;
                var ska1 = 0.075f;
                UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt1, "black", rot1, off1, ska1, meth, goname: "BoxIdTxt1");
                UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt1, "black", rot2, off2, ska1, meth, goname: "BoxIdTxt1");
                UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt2, "black", rot3, off3, ska1, meth, goname: "BoxIdTxt2");
            }
        }

    }
}