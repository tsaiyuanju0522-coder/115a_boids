using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;

using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace  BoidsPlugin
{
    class Boid
{
    public Point3d Position;
    public Vector3d Velocity;
    public Vector3d Acceleration = Vector3d.Zero;
    public double MaxSpeed;
    public double MaxForce;

    public Boid(Point3d position, Vector3d velocity, double maxSpeed, double maxForce)
    {
        Position = position;
        Velocity = velocity;
        MaxSpeed = maxSpeed;
        MaxForce = maxForce;
    }

    public void Update(Point3d min, Point3d max, List<Brep> obstacles)
    {
        Point3d previous = Position;

        Position = Position + Velocity;
        Velocity = Velocity + Acceleration;
        if (Velocity.Length > MaxSpeed)
            Velocity = Velocity * (MaxSpeed / Velocity.Length);
        Acceleration = Vector3d.Zero;
        Bounce(min, max);
        HitObstacles(previous, obstacles);
    }

    // 撞牆反彈
    void Bounce(Point3d min, Point3d max)
    {
        double x = Position.X, y = Position.Y, z = Position.Z;
        double vx = Velocity.X, vy = Velocity.Y, vz = Velocity.Z;

        BounceAxis(ref x, ref vx, min.X, max.X);
        BounceAxis(ref y, ref vy, min.Y, max.Y);
        BounceAxis(ref z, ref vz, min.Z, max.Z);

        Position = new Point3d(x, y, z);
        Velocity = new Vector3d(vx, vy, vz);
    }

    static void BounceAxis(ref double position, ref double velocity, double low, double high)
    {
        if (position > high)
        {
            position = high;
            velocity = -velocity;
        }
        else if (position < low)
        {
            position = low;
            velocity = -velocity;
        }
    }

    // 保底：上一個位置 → 現在位置的線段穿過 Brep，就退回上一個位置，並沿表面法線反彈
    void HitObstacles(Point3d previous, List<Brep> obstacles)
    {
        if (previous.DistanceTo(Position) < 1e-9) return;
        var path = new LineCurve(previous, Position);

        foreach (Brep brep in obstacles)
        {
            Curve[] overlapCurves;
            Point3d[] hitPoints;
            if (!Intersection.CurveBrep(path, brep, 1e-6, out overlapCurves, out hitPoints)) continue;
            if (hitPoints == null || hitPoints.Length == 0) continue;

            // 最靠近出發點的那個交點，才是真正撞上的位置
            Point3d hit = hitPoints.OrderBy(point => point.DistanceTo(previous)).First();

            Point3d closest;
            ComponentIndex componentIndex;
            double s, t;
            Vector3d normal;
            if (!brep.ClosestPoint(hit, out closest, out componentIndex, out s, out t, 1.0, out normal)
                || !normal.Unitize())
            {
                normal = previous - hit;
                if (!normal.Unitize()) normal = -Velocity;
                normal.Unitize();
            }

            // 法線可能朝內，讓它指向出發點那一側（鳥原本在的那一側）
            if (normal * (previous - hit) < 0) normal = -normal;

            Position = previous;
            double along = Velocity * normal;
            if (along < 0) Velocity = Velocity - 2.0 * along * normal;
            return;
        }
    }

    // 點在任何一個「實心」Brep 裡面（開放曲面沒有裡外，不算）
    public static bool IsInsideAny(Point3d point, List<Brep> obstacles)
    {
        foreach (Brep brep in obstacles)
        {
            if (brep.IsSolid && brep.IsPointInside(point, 1e-6, false)) return true;
        }
        return false;
    }

    // 規則 1：對齊　viewRadius 內鄰居的平均飛行方向 → 轉向力（沒有鄰居就是 0）
    public Vector3d Align(List<Boid> boids, double viewRadius)
    {
        Vector3d alignSum = Vector3d.Zero;
        int count = 0;

        foreach (Boid other in boids)
        {
            if (other == this) continue;
            if (Position.DistanceTo(other.Position) < viewRadius)
            {
                alignSum += other.Velocity;
                count++;
            }
        }

        if (count == 0) return Vector3d.Zero;
        return Steer(alignSum);
    }

    // 規則 2：聚集　viewRadius 內鄰居的平均位置 − 自己的位置 → 轉向力（沒有鄰居就是 0）
    public Vector3d Cohesion(List<Boid> boids, double viewRadius)
    {
        Vector3d positionSum = Vector3d.Zero;
        int count = 0;

        foreach (Boid other in boids)
        {
            if (other == this) continue;
            if (Position.DistanceTo(other.Position) < viewRadius)
            {
                positionSum += new Vector3d(other.Position);
                count++;
            }
        }

        if (count == 0) return Vector3d.Zero;
        return Steer(positionSum / count - new Vector3d(Position));
    }

    // 規則 3：分離　separationRadius 內的鄰居：(自己位置 − 對方位置) / distance²，越近躲得越用力
    public Vector3d Separation(List<Boid> boids, double separationRadius)
    {
        Vector3d awaySum = Vector3d.Zero;

        foreach (Boid other in boids)
        {
            if (other == this) continue;
            double distance = Position.DistanceTo(other.Position);
            // 兩隻疊在同一點時方向沒定義，也不能除以 0，要跳過
            if (distance > 1e-6 && distance < separationRadius)
            {
                awaySum += (Position - other.Position) / (distance * distance);
            }
        }

        return Steer(awaySum);
    }

    // 規則 4：閃避　avoidRadius 內，離每個 Brep 最近的表面點：(自己位置 − 表面點) / distance²
    public Vector3d Avoid(List<Brep> obstacles, double avoidRadius)
    {
        Vector3d awaySum = Vector3d.Zero;

        foreach (Brep brep in obstacles)
        {
            Point3d surfacePoint = brep.ClosestPoint(Position);
            double distance = Position.DistanceTo(surfacePoint);
            // 剛好貼在表面上，方向沒定義，交給保底處理
            if (distance > 1e-6 && distance < avoidRadius)
            {
                awaySum += (Position - surfacePoint) / (distance * distance);
            }
        }

        return Steer(awaySum);
    }

    // 轉向力 ＝ 期望速度 − 目前速度，長度限制在 MaxForce
    public Vector3d Steer(Vector3d desired)
    {
        if (!desired.Unitize()) return Vector3d.Zero;
        Vector3d steering = desired * MaxSpeed - Velocity;
        if (steering.Length > MaxForce)
            steering = steering * (MaxForce / steering.Length);
        return steering;
    }
}

}