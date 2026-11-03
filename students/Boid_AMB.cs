using System;
using System.Collections.Generic;
using System.Drawing;
using Rhino.Geometry;

namespace BoidArena
{
    // AMB：包圍型集體狩獵，搭配攻守平衡
    public class Boid_AMB : IBrain, IDeclareRelations
    {
        // Declare 會為整個 AMB 隊伍選出同一個主要目標。
        // 這是依隊名計算的固定偽隨機選擇，確保每隻鳥選到相同目標。
        private string _targetName = null;

        public string Name
        {
            get { return "AMB"; }
        }

        // 開賽前呼叫一次：選定一支主要狩獵目標。
        public void Declare(RelationTable table)
        {
            _targetName = null;

            if (table == null || table.AllNames == null)
                return;

            int enemyCount = 0;
            for (int i = 0; i < table.AllNames.Count; i++)
            {
                string candidate = table.AllNames[i];
                if (!string.IsNullOrEmpty(candidate) &&
                    !string.Equals(candidate, table.MyName, StringComparison.Ordinal))
                {
                    enemyCount++;
                }
            }

            if (enemyCount == 0)
                return;

            // 以隊名計算穩定雜湊值，避免每隻 Boid 各自抽到不同目標。
            int hash = 17;
            string myName = table.MyName ?? "AMB";
            for (int i = 0; i < myName.Length; i++)
                hash = unchecked(hash * 31 + myName[i]);

            int chosenIndex = (hash & 0x7fffffff) % enemyCount;
            int currentIndex = 0;

            for (int i = 0; i < table.AllNames.Count; i++)
            {
                string candidate = table.AllNames[i];

                if (string.IsNullOrEmpty(candidate) ||
                    string.Equals(candidate, table.MyName, StringComparison.Ordinal))
                    continue;

                if (currentIndex == chosenIndex)
                {
                    _targetName = candidate;
                    break;
                }

                currentIndex++;
            }

            // AMB 以所有異隊為敵軍；不改變遊戲引擎本身的擊殺規則。
            for (int i = 0; i < table.AllNames.Count; i++)
            {
                string candidate = table.AllNames[i];
                if (!string.IsNullOrEmpty(candidate) &&
                    !string.Equals(candidate, table.MyName, StringComparison.Ordinal))
                {
                    table.Enemy(candidate);
                }
            }
        }

        // 每隻鳥每一步呼叫一次：計算包圍、突襲、覓食與避險方向。
        public Vector3d Decide(
            SelfInfo self,
            List<Neighbor> neighbors,
            List<Point3d> foods,
            World world)
        {
            if (self == null || world == null)
                return Vector3d.Zero;

            Vector3d desired = Vector3d.Zero;
            Vector3d myVelocity = Flat(self.Velocity);

            if (LengthSquared(myVelocity) < 0.000001)
                myVelocity = new Vector3d(1.0, 0.0, 0.0);

            // ------------------------------------------------------------
            // 1. 優先尋找主要目標隊伍中可攻擊的敵人。
            // ------------------------------------------------------------
            Neighbor mainTarget = null;
            Neighbor nearestEnemy = null;
            double mainTargetDistance = double.MaxValue;
            double nearestEnemyDistance = double.MaxValue;

            if (neighbors != null)
            {
                for (int i = 0; i < neighbors.Count; i++)
                {
                    Neighbor n = neighbors[i];
                    if (n == null || n.InvincibleSteps > 0)
                        continue;

                    if (n.Relation != Relation.Enemy)
                        continue;

                    if (n.Distance < nearestEnemyDistance)
                    {
                        nearestEnemyDistance = n.Distance;
                        nearestEnemy = n;
                    }

                    if (!string.IsNullOrEmpty(_targetName) &&
                        string.Equals(n.Name, _targetName, StringComparison.Ordinal) &&
                        n.Distance < mainTargetDistance)
                    {
                        mainTargetDistance = n.Distance;
                        mainTarget = n;
                    }
                }
            }

            // 主要目標暫時不在視野內時，先對最近的可攻擊敵人採取防禦性反應；
            // 主要目標仍保留，不會因單隻鳥看不到它就改變全隊目標。
            Neighbor attackTarget = mainTarget != null ? mainTarget : nearestEnemy;

            if (attackTarget != null)
            {
                if (mainTarget != null)
                {
                    // 主要目標可見：計算該目標隊伍的可攻擊成員中心，
                    // 讓 AMB 成員朝目標群周圍的環形位置分散，形成包圍。
                    double sumX = 0.0;
                    double sumY = 0.0;
                    int count = 0;

                    for (int i = 0; neighbors != null && i < neighbors.Count; i++)
                    {
                        Neighbor n = neighbors[i];
                        if (n == null || n.InvincibleSteps > 0)
                            continue;

                        if (n.Relation == Relation.Enemy &&
                            string.Equals(n.Name, _targetName, StringComparison.Ordinal))
                        {
                            sumX += n.Position.X;
                            sumY += n.Position.Y;
                            count++;
                        }
                    }

                    if (count > 0)
                    {
                        Point3d enemyCenter = new Point3d(
                            sumX / count,
                            sumY / count,
                            0.0);

                        Vector3d radial = new Vector3d(
                            self.Position.X - enemyCenter.X,
                            self.Position.Y - enemyCenter.Y,
                            0.0);

                        // 若剛好位於中心，使用自己的行進方向建立初始分散方向。
                        if (LengthSquared(radial) < 0.000001)
                            radial = new Vector3d(-myVelocity.Y, myVelocity.X, 0.0);

                        radial = SafeUnit(radial);

                        // 以約 3.2 個單位的半徑建立包圍位置。
                        Point3d ringPoint = new Point3d(
                            enemyCenter.X + radial.X * 3.2,
                            enemyCenter.Y + radial.Y * 3.2,
                            0.0);

                        Vector3d toRing = FromTo(self.Position, ringPoint);

                        if (LengthSquared(toRing) > 0.20)
                        {
                            desired += SafeUnit(toRing) * 1.15;
                        }
                        else
                        {
                            // 到達包圍位置附近後，優先繞到最近敵人的背後。
                            Vector3d enemyForward = SafeUnit(Flat(attackTarget.Velocity));

                            if (LengthSquared(enemyForward) < 0.000001)
                                enemyForward = SafeUnit(FromTo(self.Position, attackTarget.Position));

                            if (LengthSquared(enemyForward) < 0.000001)
                                enemyForward = myVelocity;

                            Point3d rearPoint = new Point3d(
                                attackTarget.Position.X - enemyForward.X * 1.0,
                                attackTarget.Position.Y - enemyForward.Y * 1.0,
                                0.0);

                            Vector3d toRear = FromTo(self.Position, rearPoint);
                            desired += SafeUnit(toRear) * 1.65;

                            // 接近背後時，讓自身速度朝向敵人，爭取觸發背後擊殺判定。
                            Vector3d towardEnemy = FromTo(self.Position, attackTarget.Position);
                            desired += SafeUnit(towardEnemy) * 0.55;
                        }
                    }
                }
                else
                {
                    // 主要目標不在視野時，短暫攻擊最近的可見敵人，
                    // 權重較低，避免全隊因局部視野差異而完全放棄共同目標。
                    Vector3d toEnemy = FromTo(self.Position, attackTarget.Position);
                    desired += SafeUnit(toEnemy) * 0.65;
                }
            }

            // ------------------------------------------------------------
            // 2. 五芒星優先：尚未無敵時，朝最近的星星移動。
            // ------------------------------------------------------------
            if (self.InvincibleSteps <= 0 && self.Stars != null && self.Stars.Count > 0)
            {
                Point3d closestStar = Point3d.Unset;
                double closestStarDistance = double.MaxValue;

                for (int i = 0; i < self.Stars.Count; i++)
                {
                    Point3d star = self.Stars[i];
                    double d = Distance2D(self.Position, star);

                    if (d < closestStarDistance)
                    {
                        closestStarDistance = d;
                        closestStar = star;
                    }
                }

                if (closestStarDistance < double.MaxValue)
                {
                    Vector3d toStar = FromTo(self.Position, closestStar);
                    double starWeight = closestStarDistance < 2.0 ? 2.2 : 1.35;
                    desired += SafeUnit(toStar) * starWeight;
                }
            }

            // ------------------------------------------------------------
            // 3. 食物：維持隊伍數量。正在包圍主要目標時降低覓食權重。
            // ------------------------------------------------------------
            if (foods != null && foods.Count > 0)
            {
                Point3d closestFood = Point3d.Unset;
                double closestFoodDistance = double.MaxValue;

                for (int i = 0; i < foods.Count; i++)
                {
                    double d = Distance2D(self.Position, foods[i]);
                    if (d < closestFoodDistance)
                    {
                        closestFoodDistance = d;
                        closestFood = foods[i];
                    }
                }

                if (closestFoodDistance < double.MaxValue)
                {
                    Vector3d toFood = FromTo(self.Position, closestFood);
                    double foodWeight = mainTarget != null ? 0.30 : 0.85;
                    desired += SafeUnit(toFood) * foodWeight;
                }
            }

            // ------------------------------------------------------------
            // 4. 無敵敵人不追殺；距離太近時先拉開，避免無效碰撞。
            // ------------------------------------------------------------
            if (neighbors != null)
            {
                for (int i = 0; i < neighbors.Count; i++)
                {
                    Neighbor n = neighbors[i];
                    if (n == null || n.InvincibleSteps <= 0)
                        continue;

                    if (n.Distance < 2.8)
                    {
                        Vector3d away = new Vector3d(
                            self.Position.X - n.Position.X,
                            self.Position.Y - n.Position.Y,
                            0.0);

                        double strength = (2.8 - n.Distance) / 2.8;
                        desired += SafeUnit(away) * (1.25 * strength);
                    }
                }
            }

            // ------------------------------------------------------------
            // 5. 場地邊界：場地縮小後也要留在安全範圍內。
            // ------------------------------------------------------------
            double half = world.HalfSize;
            double margin = 1.8;
            double safeHalf = Math.Max(0.0, half - margin);

            double dx = self.Position.X - world.Center.X;
            double dy = self.Position.Y - world.Center.Y;

            if (Math.Abs(dx) > safeHalf || Math.Abs(dy) > safeHalf)
            {
                Vector3d toCenter = new Vector3d(
                    world.Center.X - self.Position.X,
                    world.Center.Y - self.Position.Y,
                    0.0);

                double edgeStrength = 1.0;
                if (Math.Abs(dx) > safeHalf + 1.0 || Math.Abs(dy) > safeHalf + 1.0)
                    edgeStrength = 1.8;

                desired += SafeUnit(toCenter) * edgeStrength;
            }

            // ------------------------------------------------------------
            // 6. 障礙物避讓：靠近矩形障礙時，朝最近邊界的反方向移動。
            // ------------------------------------------------------------
            if (world.Obstacles != null)
            {
                for (int i = 0; i < world.Obstacles.Count; i++)
                {
                    Obstacle obstacle = world.Obstacles[i];
                    if (obstacle == null)
                        continue;

                    double obstacleDistance = obstacle.DistanceTo(self.Position);
                    if (obstacleDistance < 2.2)
                    {
                        Point3d closest = obstacle.ClosestPoint(self.Position);
                        Vector3d away = new Vector3d(
                            self.Position.X - closest.X,
                            self.Position.Y - closest.Y,
                            0.0);

                        if (LengthSquared(away) < 0.000001)
                        {
                            away = new Vector3d(
                                self.Position.X - obstacle.Center.X,
                                self.Position.Y - obstacle.Center.Y,
                                0.0);
                        }

                        double strength = (2.2 - obstacleDistance) / 2.2;
                        desired += SafeUnit(away) * (1.6 * strength);
                    }
                }
            }

            // 所有計算都限制在 XY 平面，最後交給引擎限制最大轉向力。
            desired.Z = 0.0;

            if (LengthSquared(desired) < 0.000001)
                return Vector3d.Zero;

            return Steering.Seek(SafeUnit(desired), myVelocity, world);
        }

        // 將向量投影到 XY 平面。
        private static Vector3d Flat(Vector3d v)
        {
            return new Vector3d(v.X, v.Y, 0.0);
        }

        // 從起點指向終點的 2D 向量。
        private static Vector3d FromTo(Point3d from, Point3d to)
        {
            return new Vector3d(to.X - from.X, to.Y - from.Y, 0.0);
        }

        // 使用老師提供的 Steering.Unit，避免直接呼叫 Unitize。
        private static Vector3d SafeUnit(Vector3d v)
        {
            return Steering.Unit(Flat(v));
        }

        // 只計算 XY 平面的向量長度平方。
        private static double LengthSquared(Vector3d v)
        {
            return v.X * v.X + v.Y * v.Y;
        }

        // 只計算 XY 平面距離。
        private static double Distance2D(Point3d a, Point3d b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }
    }
}
