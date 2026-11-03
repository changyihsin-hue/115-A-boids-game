using System;
using System.Collections.Generic;
using Rhino.Geometry;

namespace BoidArena
{
    public class Boid_oralex : IBrain, IDeclareRelations
    {
        public string Name { get { return "oralex"; } }

        public void Declare(RelationTable table) { }

        public Vector3d Decide(SelfInfo self, List<Neighbor> neighbors, List<Point3d> foods, World world)
        {
            Vector3d heading = Steering.Unit(self.Velocity);
            Vector3d danger = Vector3d.Zero;
            Vector3d safety = Vector3d.Zero;
            Vector3d target = Vector3d.Zero;
            double bestScore = double.MaxValue;

            // 異隊都可能擊殺我；優先避開接近且有攻擊角度的對手。
            foreach (Neighbor n in neighbors)
            {
                if (n.IsFriend || n.Distance < 0.001) continue;
                Vector3d offset = n.Position - self.Position;
                Vector3d direction = Steering.Unit(offset);
                Vector3d enemyHeading = Steering.Unit(n.Velocity);
                double enemyFacesMe = -(enemyHeading * direction);
                bool behindMe = heading * direction < -0.25;

                if (self.InvincibleSteps == 0 && n.Distance < 10 &&
                    (enemyFacesMe > 0.35 || (behindMe && n.Distance < 4)))
                {
                    danger -= direction * ((10.0 - n.Distance) / 10.0);
                }

                // 瞄準對手後方及其移動方向；不追逐無敵的對手。
                if (n.InvincibleSteps == 0)
                {
                    double score = n.Distance + (1.0 - enemyHeading * direction) * 5.0;
                    if (score < bestScore)
                    {
                        bestScore = score;
                        target = offset + n.Velocity * Math.Min(n.Distance / Math.Max(world.MaxSpeed, 0.01), 4.0)
                            - enemyHeading * 1.5;
                    }
                }
            }

            // 提前遠離場地邊界，避免縮圈或轉向不及。
            double x = self.Position.X - world.Center.X;
            double y = self.Position.Y - world.Center.Y;
            double margin = Math.Min(6.0, world.HalfSize * 0.3);
            if (x > world.HalfSize - margin) safety.X -= 1;
            if (x < -world.HalfSize + margin) safety.X += 1;
            if (y > world.HalfSize - margin) safety.Y -= 1;
            if (y < -world.HalfSize + margin) safety.Y += 1;

            foreach (Obstacle obstacle in world.Obstacles)
            {
                double distance = obstacle.DistanceTo(self.Position);
                if (distance < 4.0)
                {
                    Vector3d away = self.Position - obstacle.ClosestPoint(self.Position);
                    if (away.Length < 0.001) away = self.Position - obstacle.Center;
                    safety += Steering.Unit(away) * ((4.0 - distance) / 4.0);
                }
            }

            // 危險發生時不讓攻擊或食物的方向抵消逃生方向。
            if (safety.Length > 0.001 || danger.Length > 0.001)
            {
                Vector3d escape = Steering.Unit(safety) * 2.0 + Steering.Unit(danger);
                if (escape.Length < 0.001) escape = safety.Length > 0.001 ? safety : danger;
                return Steering.Seek(escape, self.Velocity, world);
            }

            // 星星提供無敵，對存活和反擊都有幫助。
            if (self.InvincibleSteps == 0 && self.Stars.Count > 0)
            {
                Point3d nearest = self.Stars[0];
                double nearestDistance = self.Position.DistanceTo(nearest);
                foreach (Point3d star in self.Stars)
                {
                    double distance = self.Position.DistanceTo(star);
                    if (distance < nearestDistance) { nearest = star; nearestDistance = distance; }
                }
                return Steering.Seek(Route(self.Position, nearest, world), self.Velocity, world);
            }

            if (bestScore < double.MaxValue && target.Length > 0.001)
                return Steering.Seek(Route(self.Position, self.Position + target, world), self.Velocity, world);

            // 視野內沒有可攻擊的對手時尋找食物。
            if (foods.Count > 0)
            {
                Point3d nearest = foods[0];
                double nearestDistance = self.Position.DistanceTo(nearest);
                foreach (Point3d food in foods)
                {
                    double distance = self.Position.DistanceTo(food);
                    if (distance < nearestDistance) { nearest = food; nearestDistance = distance; }
                }
                return Steering.Seek(Route(self.Position, nearest, world), self.Velocity, world);
            }
            return Vector3d.Zero;
        }
        // ??????????????? A* ?????????
        // ?????????????????????
        private Vector3d Route(Point3d start, Point3d goal, World world)
        {
            if (ClearPath(start, goal, world)) return goal - start;

            const double clearance = 1.0;
            List<Point3d> nodes = new List<Point3d>();
            nodes.Add(start);
            nodes.Add(goal);
            foreach (Obstacle o in world.Obstacles)
            {
                for (int sx = -1; sx <= 1; sx += 2)
                    for (int sy = -1; sy <= 1; sy += 2)
                    {
                        Point3d corner = new Point3d(o.Center.X + sx * (o.HalfWidth + clearance),
                            o.Center.Y + sy * (o.HalfHeight + clearance), 0);
                        if (Math.Abs(corner.X - world.Center.X) < world.HalfSize - 1 &&
                            Math.Abs(corner.Y - world.Center.Y) < world.HalfSize - 1)
                            nodes.Add(corner);
                    }
            }

            int count = nodes.Count;
            double[] cost = new double[count];
            int[] previous = new int[count];
            bool[] closed = new bool[count];
            for (int i = 0; i < count; i++) { cost[i] = double.MaxValue; previous[i] = -1; }
            cost[0] = 0;
            for (int iteration = 0; iteration < count; iteration++)
            {
                int current = -1;
                double best = double.MaxValue;
                for (int i = 0; i < count; i++)
                {
                    if (closed[i] || cost[i] == double.MaxValue) continue;
                    double estimate = cost[i] + nodes[i].DistanceTo(goal);
                    if (estimate < best) { best = estimate; current = i; }
                }
                if (current < 0 || current == 1) break;
                closed[current] = true;
                for (int next = 1; next < count; next++)
                {
                    if (closed[next] || next == current) continue;
                    double candidate = cost[current] + nodes[current].DistanceTo(nodes[next]);
                    if (candidate < cost[next] && ClearPath(nodes[current], nodes[next], world))
                    {
                        cost[next] = candidate;
                        previous[next] = current;
                    }
                }
            }
            if (previous[1] < 0) return goal - start;
            int waypoint = 1;
            while (previous[waypoint] > 0) waypoint = previous[waypoint];
            return nodes[waypoint] - start;
        }

        // ???????????????????
        private bool ClearPath(Point3d a, Point3d b, World world)
        {
            foreach (Obstacle o in world.Obstacles)
            {
                double low = 0.0, high = 1.0;
                if (ClipAxis(a.X, b.X - a.X, o.Center.X - o.HalfWidth - 0.5,
                    o.Center.X + o.HalfWidth + 0.5, ref low, ref high) &&
                    ClipAxis(a.Y, b.Y - a.Y, o.Center.Y - o.HalfHeight - 0.5,
                    o.Center.Y + o.HalfHeight + 0.5, ref low, ref high)) return false;
            }
            return true;
        }

        private bool ClipAxis(double origin, double delta, double min, double max, ref double low, ref double high)
        {
            if (Math.Abs(delta) < 1e-10) return origin >= min && origin <= max;
            double first = (min - origin) / delta;
            double second = (max - origin) / delta;
            if (first > second) { double swap = first; first = second; second = swap; }
            low = Math.Max(low, first);
            high = Math.Min(high, second);
            return low <= high;
        }
    }
}
