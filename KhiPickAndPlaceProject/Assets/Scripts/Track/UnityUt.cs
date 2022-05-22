using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UnityUt : MonoBehaviour
{
    static Dictionary<string, string> hexColorTable = null;
    static Dictionary<string, Color> colorTable = null;
    static Dictionary<string, string> colorOrigin = null;
    public static int colordups = 0;
    public static int colorconflicts = 0;

    public static bool IsColorName(string name)
    {
        if (colorTable == null)
        {
            InitColorTable();
        }
        return colorTable.ContainsKey(name);
    }

    static bool ClrEqual(Color c1, Color c2)
    {
        if (c1.r != c2.r) return false;
        if (c1.g != c2.g) return false;
        if (c1.b != c2.b) return false;
        return true;
    }

    public static void AddCoreColor(string cname, Color clr)
    {
        if (colorTable.ContainsKey(cname))
        {
            var oldclr = colorTable[cname];
            if (ClrEqual(clr, oldclr))
            {
                colordups++;
                return;
            }
            colorconflicts++;
            //Debug.LogWarning($"Duplicate color {cname}"); // to be expected
            return;
        }
        colorTable[cname] = clr;
        colorOrigin[cname] = "core";
    }


    public static void AddColor(string cname, Color clr)
    {
        if (colorTable.ContainsKey(cname))
        {
            //Debug.LogWarning($"Duplicate color {cname}"); // to be expected
            var oldclr = colorTable[cname];
            if (ClrEqual(clr, oldclr))
            {
                colordups++;
                return;
            }
            colorconflicts++;
            return;
        }
        colorTable[cname] = clr;
        colorOrigin[cname] = "adhoc";
    }



    public static void InitCoreColors()
    {
        AddCoreColor("red", Color.red);
        AddCoreColor("green", Color.green);
        AddCoreColor("blue", Color.blue);
        AddCoreColor("gray", Color.gray);
        AddCoreColor("grey", Color.grey);
        AddCoreColor("cyan", Color.cyan);
        AddCoreColor("magenta", Color.magenta);
        AddCoreColor("yellow", Color.yellow);
        AddCoreColor("white", Color.white);
        AddCoreColor("black", Color.black);
        AddCoreColor("clear", Color.clear);
    }
    public static Color RgbByte(int r, int g, int b, float alpha = 1)
    {
        return new Color(r / 255f, g / 255f, b / 255f, alpha);
    }
    public static void InitColorTable()
    {

        colorTable = new Dictionary<string, Color>();
        colorOrigin = new Dictionary<string, string>();
        InitCoreColors();
        // InitXkcdColors();
        // reds
        AddColor("r", new Color(1, 0, 0));
        AddColor("red", new Color(1, 0, 0));
        AddColor("dr", new Color(0.5f, 0, 0));
        AddColor("crimsom", RgbByte(220, 20, 60));
        AddColor("coral", RgbByte(255, 127, 80));
        AddColor("firebrick", RgbByte(178, 34, 34));
        AddColor("darkred", RgbByte(139, 0, 0));
        AddColor("dirtyred", RgbByte(117, 10, 10));
        AddColor("lightred", new Color(1, 0.412f, 0.71f));
        AddColor("pink", new Color(1, 0.412f, 0.71f));
        AddColor("scarlet", new Color(1, 0.14f, 0.0f));
        // yellows
        AddColor("y", new Color(1, 1, 0));
        AddColor("yellow", new Color(1, 1, 0));
        AddColor("dy", new Color(0.5f, 0.5f, 0));
        AddColor("lightyellow", new Color(1, 1, 0.5f));
        AddColor("goldenrod", RgbByte(218, 165, 32));
        // oranges
        AddColor("orange", new Color(1, 0.5f, 0));
        AddColor("lightorange", new Color(1, 0.75f, 0));
        AddColor("darkorange", new Color(0.75f, 0.25f, 0));
        AddColor("brown", new Color(0.647f, 0.164f, 0.164f));
        AddColor("saddlebrown", new Color(0.545f, 0.271f, 0.075f));
        AddColor("darkbrown", new Color(0.396f, 0.263f, 0.129f));
        // greens
        AddColor("g", new Color(0.5f, 1, 0.5f));
        AddColor("lightgreen", new Color(0.5f, 1, 0.5f));
        AddColor("green", new Color(0, 1, 0));
        AddColor("dg", new Color(0, 0.5f, 0));
        AddColor("olive", new Color(0.5f, 0.5f, 0f));
        AddColor("darkgreen", new Color(0, 0.5f, 0));
        AddColor("darkgreen1", new Color(0.004f, 0.196f, 0.125f));
        AddColor("forestgreen", new Color(0.132f, 0.543f, 0.132f));
        AddColor("limegreen", new Color(0.195f, 0.8f, 0.195f));
        AddColor("seagreen", new Color(0.33f, 1.0f, 0.62f));
        // cyans
        AddColor("c", new Color(0, 1, 1));
        AddColor("cyan", new Color(0, 1, 1));
        AddColor("dc", new Color(0, 0.5f, 0.5f));
        AddColor("turquoise", RgbByte(64, 224, 208));
        AddColor("turquis", RgbByte(64, 224, 208));
        AddColor("teal", RgbByte(0, 128, 128));
        AddColor("aquamarine", RgbByte(128, 255, 212));
        // blues
        AddColor("b", new Color(0, 0, 1));
        AddColor("blue", new Color(0, 0, 1));
        AddColor("db", new Color(0, 0, 0.5f));
        AddColor("steelblue", new Color(0.27f, 0.51f, 0.71f));
        AddColor("lightblue", RgbByte(173, 216, 230));
        AddColor("azure", RgbByte(0, 127, 255));
        AddColor("skyblue", RgbByte(135, 206, 235));
        AddColor("darkblue", new Color(0.0f, 0.0f, 0.500f));
        AddColor("navyblue", new Color(0.0f, 0.0f, 0.398f));
        // purples
        AddColor("m", new Color(1, 0, 1));
        AddColor("magenta", new Color(1, 0, 1));
        AddColor("dm", new Color(0.5f, 0, 0.5f));
        AddColor("purple", new Color(0.5f, 0, 0.5f));
        AddColor("violet", new Color(0.75f, 0, 0.75f));
        AddColor("indigo", RgbByte(43, 34, 170));
        AddColor("deeppurple", new Color(0.4f, 0, 0.4f));
        AddColor("darkpurple", RgbByte(48, 25, 52));
        AddColor("phlox", RgbByte(223, 0, 255));
        AddColor("mauve", RgbByte(224, 176, 255));
        AddColor("fuchsia", RgbByte(255, 0, 255));
        AddColor("lilac", RgbByte(200, 162, 200));
        // whites and grays
        AddColor("w", new Color(1, 1, 1));
        AddColor("white", new Color(1, 1, 1));
        AddColor("chinawhite", new Color(0.937f, 0.910f, 0.878f));
        AddColor("clear", Color.clear);
        AddColor("silver", RgbByte(192, 192, 192));
        AddColor("lightgrey", RgbByte(211, 211, 211));
        AddColor("lightgray", RgbByte(211, 211, 211));
        AddColor("slategray", RgbByte(112, 128, 144));
        AddColor("slategrey", RgbByte(112, 128, 144));
        AddColor("darkslategray", RgbByte(74, 85, 83));
        AddColor("darkslategrey", RgbByte(74, 85, 83));
        AddColor("darkgray", RgbByte(105, 105, 105));
        AddColor("darkgrey", RgbByte(105, 105, 105));
        AddColor("dimgray", RgbByte(105, 105, 105));
        AddColor("dimgrey", RgbByte(105, 105, 105));
        AddColor("grey", RgbByte(128, 128, 128));
        AddColor("gray", RgbByte(128, 128, 128));
        AddColor("blk", new Color(0, 0, 0));
        AddColor("black", new Color(0, 0, 0));
        var ncnt = colorTable.Count;
        var msg = $"Inited color table count:{ncnt} dups:{colordups} conflicts:{colorconflicts}";
    }
    static string[] dcolorseq = { "dr", "dg", "db", "dm", "dy", "dc" };
    static string[] colorseq = { "r", "g", "b", "m", "y", "c" };
    public static string GetColorBySeq(int idx, bool dark = true)
    {
        string name;
        if (dark)
        {
            name = dcolorseq[idx % dcolorseq.Length];
        }
        else
        {
            name = colorseq[idx % colorseq.Length];
        }
        return name;
    }
    public static Color GetColorByName(string name)
    {
        if (!IsColorName(name))
        {
            Debug.LogError($"color {name} not defined in colortable");
            return Color.gray;
        }
        return colorTable[name];
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }
    public static void AddFltTextMeshGameObject(GameObject go, Vector3 pt, string text, string colorname, FltTextRotate rotmeth = FltTextRotate.specified, float xrot = 0, float yrot = 0, float zrot = 0, float xoff = 0, float yoff = 0, float zoff = 0, bool wps=true)
    {
        var ngo = new GameObject("FltText");
        ngo.transform.rotation = Quaternion.Euler(xrot,yrot,zrot);
        ngo.transform.position = new Vector3(xoff,yoff,zoff);
        ngo.transform.localScale = new Vector3(1,1,1);
        ngo.transform.SetParent(go.transform, worldPositionStays:wps);
        var txgo = new GameObject("TextMesh");
        txgo.transform.SetParent(ngo.transform,worldPositionStays:false);
        AddFltTextMeshComponent(txgo, Vector3.zero, text, colorname, rotmeth, 0,0,0, xoff:0, yoff:0, zoff:0,wps:false);
        //AddFltTextMeshComponent(txgo, Vector3.zero, text, colorname, rotmeth, 0, 0, 0, xoff: xoff, yoff: yoff, zoff: zoff);
        //var cgo = CreateSphere(ngo, "limegreen", size: 0.2f, wps: false);
        //cgo.name = "sphere";
    }

    public enum FltTextImplE { TextMesh, GUIText, TextPro };
    public static FltTextImplE fltTextImpl = FltTextImplE.TextMesh;
    public enum FltTextRotate { specified, mainCam, sceneCam }
    public static void AddFltTextMeshComponent(GameObject go, Vector3 pt, string text, string colorname, FltTextRotate rotmeth=FltTextRotate.specified, float xrot = 0, float yrot = 0, float zrot = 0, float xoff = 0, float yoff = 0, float zoff = 0,bool wps=true)
    {
        switch (fltTextImpl)
        {
            default:
            case FltTextImplE.TextMesh:
                {
                    var tm = go.AddComponent<TextMesh>();
                    int linecount = text.Split('\n').Length-1;
                    tm.text = text;
                    tm.fontSize = 12;
                    tm.anchor = TextAnchor.UpperCenter;
                    float sfak = 0.1f;
                    tm.transform.localScale = new Vector3(sfak, sfak, sfak);
                    switch(rotmeth)
                    {
                        case FltTextRotate.specified:
                            tm.transform.Rotate(xrot, yrot, zrot);
                            break;
                        case FltTextRotate.mainCam:
                        case FltTextRotate.sceneCam:
                            tm.transform.rotation = Quaternion.LookRotation(Camera.main.transform.forward);
                            break;
                    }
                    tm.transform.localPosition = pt + new Vector3(xoff, sfak * yoff + linecount * 0.25f, zoff);
                    tm.transform.SetParent(go.transform,worldPositionStays:wps);
                    tm.color = GetColorByName(colorname);
                    break;
                }
            case FltTextImplE.GUIText:
                {
                    Vector2 worldPoint = Camera.main.WorldToScreenPoint(go.transform.position);
                    GUI.Label(new Rect(worldPoint.x - 100, (Screen.height - worldPoint.y) - 50, 200, 100), text);
                    break;
                }
            case FltTextImplE.TextPro:
                {
                    var tm = go.AddComponent<TMPro.TextMeshPro>();
                    int linecount = text.Split('\n').Length-1;
                    //tm.text = "<mark=#000000>"+text+"</mark>"; // this only works with an alpha of less than 1
                    tm.text = text;
                    tm.fontSize = 12;
                    tm.alignment = TMPro.TextAlignmentOptions.Center;
                    float sfak = 0.1f;
                    tm.transform.localScale = new Vector3(sfak, sfak, sfak);
                    switch (rotmeth)
                    {
                        case FltTextRotate.specified:
                            tm.transform.Rotate(xrot, yrot, zrot);
                            break;
                        case FltTextRotate.mainCam:
                        case FltTextRotate.sceneCam:
                            tm.transform.rotation = Quaternion.LookRotation(Camera.main.transform.forward);
                            break;
                    }
                    tm.transform.localPosition = pt + new Vector3(xoff, sfak * yoff + linecount * 0.25f, zoff);
                    tm.transform.SetParent(go.transform, worldPositionStays: wps);
                    tm.color = GetColorByName(colorname);
                    tm.alpha = 1.0f; // this has to stay at the end or it gets overwritten !!
                    break;
                }
        }
    }

    //public static Color mmcolor = Color.white;
    public static GameObject CreateSphere(GameObject parent, string cname="white", float size = 0.5f, bool wps = true)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.transform.localScale = new Vector3(size, size, size);
        if (parent != null)
        {
            go.transform.SetParent(parent.transform, worldPositionStays: wps);
        }
        var material = go.GetComponent<Renderer>().material;
        material.color = GetColorByName(cname);
        return go;
    }
    public static GameObject CreateCube(GameObject parent, string cname = "white", float size = 0.5f, bool wps = true)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.transform.localScale = new Vector3(size, size, size);
        if (parent != null)
        {
            go.transform.SetParent(parent.transform, worldPositionStays: wps);
        }
        var material = go.GetComponent<Renderer>().material;
        material.color = GetColorByName(cname);
        return go;
    }


    static System.Random random = new System.Random(1234);

    public static string GetRandomColorString()
    {
        var i = random.Next(4);
        switch (i)
        {
            default:
            case 0: return "red";
            case 1: return "green";
            case 2: return "blue";
            case 3: return "black";
        }
    }

}
