
using System;
using System.Collections.Generic;
using Rhino.Geometry;

namespace BoidArena
{
    public class Boid_ICcard : IBrain, IDeclareRelations
    {
        public string Name { get { return "ICcard"; } }

        private const double EPS = 1e-8;

        // 保留預設身份：同隊友軍，異隊敵軍
        public void Declare(RelationTable table) { }

        // =====================================================
        // 1. 主要行為決策
        // =====================================================

        public Vector3d Decide(SelfInfo self, List<Neighbor> neighbors,
                               List<Point3d> foods, World world)
        {
            Point3d me = self.Position;
            if (!Valid(me) || !Valid(world.Center) ||
                !OK(world.HalfSize) || world.HalfSize <= EPS)
                return Vector3d.Zero;

            Vector3d vel = self.Velocity;
            vel.Z = 0;
            if (!Valid(vel)) vel = Vector3d.Zero;

            Vector3d forward = Steering.Unit(vel);
            int count = neighbors == null ? 0 : neighbors.Count;

            // -------------------------------------------------
            // 2. 蒐集友軍、敵軍與背後威脅
            // -------------------------------------------------

            int friends = 1, teamKinds = 0, nearest = -1;
            int[] ids = new int[10], nums = new int[10], killable = new int[10];

            double sx = me.X, sy = me.Y;
            Vector3d sumVel = vel, escape = Vector3d.Zero;
            double nearestD = double.MaxValue;

            for (int i = 0; i < count; i++)
            {
                Neighbor n = neighbors[i];
                if (!Valid(n.Position)) continue;

                double d = Dist(me, n.Position);

                if (n.IsFriend)
                {
                    friends++;
                    sx += n.Position.X;
                    sy += n.Position.Y;

                    if (Valid(n.Velocity))
                        sumVel += Flat(n.Velocity);

                    continue;
                }

                // 統計各隊數量
                int k = 0;
                while (k < teamKinds && ids[k] != n.Team) k++;

                if (k == teamKinds && teamKinds < 10)
                    ids[teamKinds++] = n.Team;

                if (k < 10)
                {
                    nums[k]++;
                    if (n.InvincibleSteps <= 0) killable[k]++;
                }

                if (d < nearestD)
                {
                    nearestD = d;
                    nearest = i;
                }

                // 偵測後方追擊者，無敵時不必逃跑
                if (self.InvincibleSteps > 0 || d >= 5 || d <= EPS)
                    continue;

                Vector3d toEnemy = Steering.Unit(Dir(me, n.Position));
                Vector3d enemyForward = Valid(n.Velocity)
                    ? Steering.Unit(Flat(n.Velocity)) : Vector3d.Zero;

                bool rear = Dot(forward, toEnemy) < -0.45 &&
                            Dot(enemyForward, toEnemy) < -0.25;

                bool danger = n.InvincibleSteps > 0 && d < 2.3;

                if (rear || danger)
                {
                    double strength = (5 - d) / (d + 0.5);
                    if (danger) strength *= 1.5;
                    escape -= toEnemy * strength;
                }
            }

            Point3d center = new Point3d(sx / friends, sy / friends, 0);
            Vector3d cohesion = Dir(me, center);
            Vector3d alignment = Steering.Unit(sumVel);

            // -------------------------------------------------
            // 3. 尋找數量最多、仍有可擊殺成員的敵隊
            // -------------------------------------------------

            int targetTeam = -1, largest = 0;

            for (int i = 0; i < teamKinds; i++)
            {
                if (killable[i] > 0 && nums[i] > largest)
                {
                    largest = nums[i];
                    targetTeam = ids[i];
                }
            }

            int target = -1;
            double bestScore = double.MaxValue;

            for (int i = 0; i < count; i++)
            {
                Neighbor n = neighbors[i];

                if (n.IsFriend || n.Team != targetTeam ||
                    n.InvincibleSteps > 0 || !Valid(n.Position))
                    continue;

                double score = Dist(me, n.Position);

                // 正面接近較危險，優先挑選容易背襲的敵人
                if (Valid(n.Velocity) &&
                    Dot(Steering.Unit(Flat(n.Velocity)),
                        Dir(n.Position, me)) > 0)
                    score += 1.5;

                if (score < bestScore)
                {
                    bestScore = score;
                    target = i;
                }
            }

            // -------------------------------------------------
            // 4. 限制局部突擊比例為三分之一
            // -------------------------------------------------

            bool attack = false;
            int quota = friends / 3;

            if (target >= 0 && quota > 0)
            {
                Point3d goal = neighbors[target].Position;
                double myD = Dist(me, goal);
                int ahead = 0;

                for (int i = 0; i < count; i++)
                {
                    Neighbor n = neighbors[i];
                    if (!n.IsFriend || !Valid(n.Position)) continue;

                    double d = Dist(n.Position, goal);

                    if (d < myD - EPS ||
                        (Math.Abs(d - myD) <= EPS &&
                         (n.Position.X < me.X - EPS ||
                          (Math.Abs(n.Position.X - me.X) <= EPS &&
                           n.Position.Y < me.Y - EPS))))
                        ahead++;
                }

                attack = ahead < quota && Exposure(goal, world) <= 5;
            }

            // -------------------------------------------------
            // 5. 搜尋食物與五芒星
            // -------------------------------------------------

            int food = -1, star = -1;
            double foodD = double.MaxValue, starD = double.MaxValue;
            double foodScore = double.MaxValue, starScore = double.MaxValue;

            if (foods != null)
                for (int i = 0; i < foods.Count; i++)
                {
                    if (!Valid(foods[i])) continue;

                    double d = Dist(me, foods[i]);
                    double score = d + Exposure(foods[i], world) * 2;

                    if (score < foodScore)
                    {
                        foodScore = score;
                        foodD = d;
                        food = i;
                    }
                }

            if (self.InvincibleSteps <= 0 && self.Stars != null)
                for (int i = 0; i < self.Stars.Count; i++)
                {
                    if (!Valid(self.Stars[i])) continue;

                    double d = Dist(me, self.Stars[i]);
                    double score = d + Exposure(self.Stars[i], world) * 2;

                    if (score < starScore)
                    {
                        starScore = score;
                        starD = d;
                        star = i;
                    }
                }

            // -------------------------------------------------
            // 6. V 字箭頭防禦陣型
            // -------------------------------------------------

            Vector3d front = alignment;

            if (nearest >= 0)
                front = Steering.Unit(
                    Dir(center, neighbors[nearest].Position));

            if (front.Length < EPS)
                front = Steering.Unit(Dir(center, world.Center));

            if (front.Length < EPS) front = forward;
            if (front.Length < EPS) front = new Vector3d(1, 0, 0);

            Vector3d side = new Vector3d(-front.Y, front.X, 0);
            Vector3d relative = Dir(center, me);

            double longitudinal = Dot(relative, front);
            double lateral = Dot(relative, side);
            double error = Clamp(
                0.65 - Math.Abs(lateral) * 0.65 - longitudinal,
                -2.3, 2.3);

            Vector3d formation = Vector3d.Zero;

            if (friends >= 3)
            {
                formation = front * (error * 1.2);

                // 中央成員向兩翼分散，避免擁擠
                if (friends >= 6 && Math.Abs(lateral) < 0.35)
                    formation += side * (lateral < 0 ? -0.4 : 0.4);
            }

            // -------------------------------------------------
            // 7. 整合移動優先順序
            // -------------------------------------------------

            Vector3d desired = formation;

            if (friends >= 2)
            {
                desired += Steering.Unit(cohesion) *
                           (world.Phase == 3 ? 1.1 : 0.7);
                desired += alignment * 0.35;
            }

            if (escape.Length > EPS)
            {
                // 最高優先：逃離背後威脅
                desired += Steering.Unit(escape) * 10;
            }
            else if (star >= 0 && (food < 0 || starD <= foodD + 3))
            {
                // 優先取得無敵能力
                desired += Steering.Unit(
                    Dir(me, self.Stars[star])) * 6;
            }
            else if (food >= 0 &&
                     !(world.Phase == 3 && attack && bestScore < 4))
            {
                // 收集食物以增加隊伍數量
                desired += Steering.Unit(Dir(me, foods[food])) * 5;
            }
            else if (attack)
            {
                // 突擊者繞向目標後方
                Neighbor n = neighbors[target];
                Vector3d nv = Valid(n.Velocity)
                    ? Flat(n.Velocity) : Vector3d.Zero;

                Vector3d nf = Steering.Unit(nv);

                Point3d predicted = n.Position + nv * 1.5;
                Point3d goal = predicted - nf * 1.2;

                if (Dist(me, n.Position) < 1.5 &&
                    Dot(nf, Dir(n.Position, me)) < -0.25)
                    goal = predicted;

                desired += Steering.Unit(Dir(me, goal)) * 5.5;
            }
            else
            {
                // 其他成員維持朝外防禦
                desired += front * 0.3;

                if (nearest >= 0 && nearestD < 3.2 &&
                    self.InvincibleSteps <= 0)
                    desired += Steering.Unit(
                        Dir(neighbors[nearest].Position, me)) *
                        (3.2 - nearestD) * 1.4;
            }

            // -------------------------------------------------
            // 8. 迴避場地邊界與障礙物
            // -------------------------------------------------

            desired += AvoidWalls(me, vel, world) *
                       (world.Phase == 2 ? 11 : 8);

            desired += AvoidObstacles(me, vel, world) * 11;
            desired.Z = 0;

            if (!Valid(desired)) return Vector3d.Zero;

            // 沒有方向時保持慣性，必要時使用固定種子亂數
            if (desired.Length < EPS)
            {
                desired = vel;

                if (desired.Length < EPS)
                {
                    double angle = self.Rng == null ? 0 :
                        self.Rng.NextDouble() * Math.PI * 2;

                    desired = new Vector3d(
                        Math.Cos(angle), Math.Sin(angle), 0);
                }
            }

            Vector3d result = Steering.Seek(desired, vel, world);

            return Valid(result)
                ? new Vector3d(result.X, result.Y, 0)
                : Vector3d.Zero;
        }

        // =====================================================
        // 9. 共用數學工具
        // =====================================================

        private static bool OK(double x)
        {
            return !double.IsNaN(x) && !double.IsInfinity(x);
        }

        private static bool Valid(Point3d p)
        {
            return OK(p.X) && OK(p.Y);
        }

        private static bool Valid(Vector3d v)
        {
            return OK(v.X) && OK(v.Y);
        }

        private static Vector3d Flat(Vector3d v)
        {
            return new Vector3d(v.X, v.Y, 0);
        }

        private static Vector3d Dir(Point3d a, Point3d b)
        {
            return new Vector3d(b.X - a.X, b.Y - a.Y, 0);
        }

        private static double Dist(Point3d a, Point3d b)
        {
            return Dir(a, b).Length;
        }

        private static double Dot(Vector3d a, Vector3d b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        private static double Clamp(double x, double a, double b)
        {
            return Math.Max(a, Math.Min(b, x));
        }

        // =====================================================
        // 10. 邊界與障礙物危險程度
        // =====================================================

        private static double Exposure(Point3d p, World w)
        {
            double x = w.HalfSize - Math.Abs(p.X - w.Center.X);
            double y = w.HalfSize - Math.Abs(p.Y - w.Center.Y);

            double risk = Math.Max(0, 1.6 - x) * 2 +
                          Math.Max(0, 1.6 - y) * 2;

            // 額外避開角落
            if (x < 2 && y < 2) risk += 3;

            if (w.Obstacles != null)
                foreach (var o in w.Obstacles)
                {
                    double d = o.DistanceTo(p);
                    if (OK(d) && d < 1.4)
                        risk += (1.4 - d) * 2;
                }

            return Math.Min(risk, 20);
        }

        // =====================================================
        // 11. 場地邊界迴避
        // =====================================================

        private static Vector3d AvoidWalls(
            Point3d p, Vector3d v, World w)
        {
            double x = p.X + v.X * 4 - w.Center.X;
            double y = p.Y + v.Y * 4 - w.Center.Y;

            double h = w.HalfSize;
            double m = Math.Min(2.3, Math.Max(0.35, h * 0.42));

            double fx = Math.Max(0, m - (x + h)) / m -
                        Math.Max(0, m - (h - x)) / m;

            double fy = Math.Max(0, m - (y + h)) / m -
                        Math.Max(0, m - (h - y)) / m;

            return new Vector3d(fx, fy, 0);
        }

        // =====================================================
        // 12. 矩形障礙物迴避
        // =====================================================

        private static Vector3d AvoidObstacles(
            Point3d p, Vector3d v, World w)
        {
            Vector3d force = Vector3d.Zero;
            if (w.Obstacles == null) return force;

            Point3d future = p + v * 3.5;

            foreach (var o in w.Obstacles)
            {
                double d = o.DistanceTo(future);
                if (!OK(d) || d >= 1.3) continue;

                Vector3d away = Dir(o.ClosestPoint(future), future);

                // 碰到矩形時沿最短出口離開
                if (d <= EPS)
                {
                    double x = future.X - o.Center.X;
                    double y = future.Y - o.Center.Y;

                    if (o.HalfWidth - Math.Abs(x) <
                        o.HalfHeight - Math.Abs(y))
                        away = new Vector3d(x >= 0 ? 1 : -1, 0, 0);
                    else
                        away = new Vector3d(0, y >= 0 ? 1 : -1, 0);
                }

                force += Steering.Unit(away) *
                         Clamp((1.3 - d) / 1.3, 0, 1);
            }

            return force;
        }
    }
}
