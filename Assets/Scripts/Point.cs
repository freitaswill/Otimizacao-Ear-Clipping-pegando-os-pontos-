using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class Point : MonoBehaviour, IPointerClickHandler
{
    public PointManager manager;
    
    public void OnPointerClick(PointerEventData data)
    {
        if (manager.mode == Mode.Select)
        {
            manager.SelectPoint(transform);
        }
    }
}
