using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace KhiDemo
{
    public class MmBox : MonoBehaviour
    {
        MagneMotion mmt;
        GameObject geomgo;
        GameObject formgo;
        public enum BoxForm { CubeBased, Prefab }
        static float boxsize = 0.2f;
        BoxForm boxform;
        string boxid;


        public static MmBox ConstructBox(MagneMotion mmt, string boxid)
        {
            var sname1 = $"sledid:{boxid}";
            var boxgeomgo = new GameObject(sname1);
            boxgeomgo.transform.position = Vector3.zero;
            boxgeomgo.transform.rotation = Quaternion.identity;
            var box = boxgeomgo.AddComponent<MmBox>();
            var boxform = mmt.boxForm;
            box.mmt = mmt;
            box.boxid = boxid;
            box.ConstructForm(boxform);
            boxgeomgo.transform.SetParent(mmt.mmtgo.transform, worldPositionStays: true);
            return box;
            //Debug.Log($"makesled pathnum:{pathnum} dist:{pathdist:f1} pt:{sledgeomgo.transform.position:f1}");
        }
        // Start is called before the first frame update
        void Start()
        {

        }

        public void ConstructForm(BoxForm boxform)
        {
            this.boxform = boxform;
            formgo = new GameObject("boxform");
            switch (this.boxform)
            {
                case BoxForm.CubeBased:
                    {

                        var go1 = UnityUt.CreateCube(formgo, "yellow", size: boxsize);
                        go1.name = $"box";
                        // 7x5.4x4.3.5
                        go1.transform.position = new Vector3(0.0f, 0.0f, -0.16f);
                        go1.transform.localScale = new Vector3(0.56f, 0.32f, 0.28f);


                        break;
                    }
                case BoxForm.Prefab:
                    {
                        var prefab1 = (GameObject)Resources.Load("Prefabs/Box1");
                        var go1 = Instantiate<GameObject>(prefab1);
                        go1.name = $"box";
                        // 7x5.4x4.3.5
                        go1.transform.parent = formgo.transform;
                        go1.transform.position = new Vector3(0.0f, 0.0f, -0.16f);
                        go1.transform.localRotation = Quaternion.Euler(180, 90, -90);
                        go1.transform.localScale = new Vector3(8, 8, 8);

                        break;
                    }
            }

            AddBoxIdToBoxForm();

            if (mmt.useMeters)
            {
                var u2m = mmt.UnitsToMeters;
                formgo.transform.localScale = new Vector3(u2m, u2m, u2m);
            }
            formgo.transform.SetParent(gameObject.transform, worldPositionStays: true);
            //Debug.Log($"ConstructSledForm pathnum:{pathnum} dist:{pathdist:f1} pt:{formgo.transform.position:f1}");
        }


        void AddBoxIdToBoxForm()
        {
            if (boxid != "")
            {
                var rot1 = new Vector3(0, 90, -90);
                var rot2 = -rot1;
                var off1 = new Vector3(-0.27f, 0, -0.12f);
                var off2 = new Vector3(+0.27f, 0, -0.12f);
                var txt = $"{boxid}";
                var meth = UnityUt.FltTextImpl.TextPro;
                UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt, "yellow", rot1, off1, meth);
                UnityUt.AddFltTextMeshGameObject(formgo, Vector3.zero, txt, "yellow", rot2, off2, meth);
            }
        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}