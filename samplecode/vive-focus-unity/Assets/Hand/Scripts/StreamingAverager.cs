using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StreamingAverager {
    public int avgSize = 16;
    public Vector3 defaultPoint = new Vector3(0, 0, 0);
    private Vector3 total;
    private LinkedList<Vector3> points;

    public StreamingAverager(Vector3 defaultPoint, int avgSize = 16)
    {
        points = new LinkedList<Vector3>();
        this.defaultPoint = defaultPoint;
        this.avgSize = avgSize;
    }

    public void addEmpty()
    {
        if (points.Count > 0)
        {
            total -= points.First.Value;
            points.RemoveFirst();
        }
    }

    public void addPoint(Vector3 value)
    {
        if (double.IsNaN(value.x) || double.IsNaN(value.y) || double.IsNaN(value.z)) return;
        points.AddLast(value);
        total += value;
        if (points.Count > avgSize)
        {
            total -= points.First.Value;
            points.RemoveFirst();
        }
        if (double.IsNaN(total.x) || double.IsNaN(total.y)  ||double.IsNaN(total.z) )
        {
            // recover from invalid point
            points.Clear();
            total = Vector3.zero;
        }
    }

    public Vector3 Point
    {
        get
        {
            if (points.Count == 0)
            {
                return defaultPoint;
            }
            return total / points.Count;
        }
    }

    public int Count
    {
        get
        {
            return points.Count;
        }
    }
}
