
using System;
using System.Collections.Generic;
using System.Drawing;
using Rhino.Geometry;

namespace BoidArena
{
    public class Boid_ICcard : IBrain, IDeclareRelations
    {
        // ============================================================
        // 0. 基本設定
        // ============================================================

        public string Name
        {
            get { return "ICcard"; }
        }

        // 數值運算使用的最小容許值
        private const double EPS = 0.00000001;

        // 開賽前設定其他隊伍的身份
        public void Declare(RelationTable table)
        {
            // 保留預設關係：
            // 同隊視為友軍，異隊視為敵軍。
            // 真正的隊伍編號由引擎決定，無法在此改為 999。
            // 修改身份標籤也無法偽裝自己的名字或隊伍編號。
        }

        // ============================================================
        // 1. 主要決策程序
        // ============================================================

        public Vector3d Decide(
            SelfInfo self,
            List<Neighbor> neighbors,
            List<Point3d> foods,
            World world)
        {
            // --------------------------------------------------------
            // 1-1. 資料讀取與基本防呆
            // --------------------------------------------------------

            Point3d me = self.Position;

            if (!Finite(me.X) || !Finite(me.Y) ||
                !Finite(world.Center.X) ||
                !Finite(world.Center.Y) ||
                !Finite(world.HalfSize) ||
                world.HalfSize <= EPS)
            {
                return Vector3d.Zero;
            }

            Vector3d velocity = new Vector3d(
                self.Velocity.X,
                self.Velocity.Y,
                0
            );

            if (!Finite(velocity.X) || !Finite(velocity.Y))
            {
                velocity = Vector3d.Zero;
            }

            Vector3d forward = Steering.Unit(velocity);

            int neighborCount =
                neighbors == null ? 0 : neighbors.Count;

            // --------------------------------------------------------
            // 1-2. 收集友軍與敵軍資訊
            // --------------------------------------------------------

            // 群體統計包含自己
            int friendCount = 1;

            double sumX = me.X;
            double sumY = me.Y;

            double sumVX = velocity.X;
            double sumVY = velocity.Y;

            // 本場最多十隊，記錄視野內各隊人數
            int[] teamIds = new int[10];
            int[] teamCounts = new int[10];
            int[] vulnerableCounts = new int[10];

            int teamKinds = 0;

            // 記錄最近的非同隊成員
            int nearestEnemyIndex = -1;
            double nearestEnemyDistance = double.MaxValue;

            // 記錄需要逃跑的方向
            Vector3d escape = Vector3d.Zero;

            for (int i = 0; i < neighborCount; i++)
            {
                Neighbor n = neighbors[i];

                if (!Finite(n.Position.X) ||
                    !Finite(n.Position.Y))
                {
                    continue;
                }

                double dx = n.Position.X - me.X;
                double dy = n.Position.Y - me.Y;

                double d = Math.Sqrt(dx * dx + dy * dy);

                // 同隊成員加入群體統計
                if (n.IsFriend)
                {
                    sumX += n.Position.X;
                    sumY += n.Position.Y;
                    friendCount++;

                    if (Finite(n.Velocity.X) &&
                        Finite(n.Velocity.Y))
                    {
                        sumVX += n.Velocity.X;
                        sumVY += n.Velocity.Y;
                    }

                    continue;
                }

                // 統計不同隊伍的人數
                int index = 0;

                while (index < teamKinds &&
                       teamIds[index] != n.Team)
                {
                    index++;
                }

                if (index == teamKinds && teamKinds < 10)
                {
                    teamIds[index] = n.Team;
                    teamKinds++;
                }

                if (index < 10)
                {
                    teamCounts[index]++;

                    // 無敵成員不能擊殺，不列入可擊殺人數
                    if (n.InvincibleSteps <= 0)
                    {
                        vulnerableCounts[index]++;
                    }
                }

                // 找出最近的非同隊成員
                if (d < nearestEnemyDistance)
                {
                    nearestEnemyDistance = d;
                    nearestEnemyIndex = i;
                }

                // ----------------------------------------------------
                // 1-3. 背後追擊偵測
                // ----------------------------------------------------

                // 自己無敵時，不必因為可能被擊殺而逃跑
                if (self.InvincibleSteps > 0)
                {
                    continue;
                }

                if (d < 5.0 && d > EPS)
                {
                    Vector3d toEnemy = new Vector3d(
                        dx / d,
                        dy / d,
                        0
                    );

                    Vector3d enemyVelocity = new Vector3d(
                        n.Velocity.X,
                        n.Velocity.Y,
                        0
                    );

                    if (!Finite(enemyVelocity.X) ||
                        !Finite(enemyVelocity.Y))
                    {
                        enemyVelocity = Vector3d.Zero;
                    }

                    Vector3d enemyForward =
                        Steering.Unit(enemyVelocity);

                    // 負值表示敵人位於自己的後方
                    double rearDot = Dot(forward, toEnemy);

                    // 正值表示敵人的朝向對準自己
                    double chasingDot =
                        -Dot(enemyForward, toEnemy);

                    bool rearThreat =
                        rearDot < -0.45 &&
                        chasingDot > 0.25;

                    // 無敵敵人距離太近時也需要避讓
                    bool invincibleThreat =
                        n.InvincibleSteps > 0 && d < 2.3;

                    if (rearThreat || invincibleThreat)
                    {
                        double urgency =
                            (5.0 - d) / (d + 0.5);

                        if (invincibleThreat)
                        {
                            urgency *= 1.5;
                        }

                        // 累加遠離危險來源的方向
                        escape += new Vector3d(
                            -toEnemy.X * urgency,
                            -toEnemy.Y * urgency,
                            0
                        );
                    }
                }
            }

            // --------------------------------------------------------
            // 2. 計算群體中心與平均移動方向
            // --------------------------------------------------------

            double centerX = sumX / friendCount;
            double centerY = sumY / friendCount;

            Vector3d cohesion = new Vector3d(
                centerX - me.X,
                centerY - me.Y,
                0
            );

            Vector3d alignment = Steering.Unit(
                new Vector3d(sumVX, sumVY, 0)
            );

            // --------------------------------------------------------
            // 3. 找出視野內數量最多的非同隊隊伍
            // --------------------------------------------------------

            int targetTeam = -1;
            int largestCount = 0;

            for (int i = 0; i < teamKinds; i++)
            {
                // 完全沒有可擊殺成員時略過
                if (vulnerableCounts[i] <= 0)
                {
                    continue;
                }

                if (teamCounts[i] > largestCount)
                {
                    largestCount = teamCounts[i];
                    targetTeam = teamIds[i];
                }
            }

            // 在主要目標隊伍中尋找合適的攻擊對象
            int targetIndex = -1;
            double bestTargetScore = double.MaxValue;

            if (targetTeam >= 0)
            {
                for (int i = 0; i < neighborCount; i++)
                {
                    Neighbor n = neighbors[i];

                    if (n.IsFriend ||
                        n.Team != targetTeam ||
                        n.InvincibleSteps > 0)
                    {
                        continue;
                    }

                    if (!Finite(n.Position.X) ||
                        !Finite(n.Position.Y))
                    {
                        continue;
                    }

                    double d = Distance(me, n.Position);
                    double score = d;

                    Vector3d enemyVelocity = new Vector3d(
                        n.Velocity.X,
                        n.Velocity.Y,
                        0
                    );

                    if (Finite(enemyVelocity.X) &&
                        Finite(enemyVelocity.Y))
                    {
                        Vector3d enemyForward =
                            Steering.Unit(enemyVelocity);

                        Vector3d enemyToMe = new Vector3d(
                            me.X - n.Position.X,
                            me.Y - n.Position.Y,
                            0
                        );

                        // 位於敵人正面時增加追擊成本
                        // 優先選擇較容易從背後接近的目標
                        if (Dot(enemyForward, enemyToMe) > 0)
                        {
                            score += 1.5;
                        }
                    }

                    if (score < bestTargetScore)
                    {
                        bestTargetScore = score;
                        targetIndex = i;
                    }
                }
            }

            // --------------------------------------------------------
            // 4. 分配局部攻擊名額
            // --------------------------------------------------------

            bool canAttack = false;

            if (targetIndex >= 0)
            {
                // 向下取整，確保局部名額不超過三分之一
                int attackQuota = friendCount / 3;

                if (attackQuota > 0)
                {
                    Point3d targetPosition =
                        neighbors[targetIndex].Position;

                    double myDistance =
                        Distance(me, targetPosition);

                    int closerFriends = 0;

                    // 越接近目標的友軍，越有機會成為突擊者
                    for (int i = 0; i < neighborCount; i++)
                    {
                        Neighbor n = neighbors[i];

                        if (!n.IsFriend ||
                            !Finite(n.Position.X) ||
                            !Finite(n.Position.Y))
                        {
                            continue;
                        }

                        double friendDistance =
                            Distance(n.Position, targetPosition);

                        bool isAhead =
                            friendDistance < myDistance - EPS;

                        // 距離相同時使用座標決定先後
                        // 避免所有成員都獲得相同排名
                        if (Math.Abs(friendDistance -
                                     myDistance) <= EPS)
                        {
                            isAhead =
                                n.Position.X < me.X - EPS ||
                                (Math.Abs(n.Position.X - me.X)
                                    <= EPS &&
                                 n.Position.Y < me.Y - EPS);
                        }

                        if (isAhead)
                        {
                            closerFriends++;
                        }
                    }

                    canAttack = closerFriends < attackQuota;

                    // 不主動追擊過度靠近障礙物或邊界的目標
                    if (Exposure(targetPosition, world) > 5.0)
                    {
                        canAttack = false;
                    }
                }
            }

            // --------------------------------------------------------
            // 5. 搜尋食物
            // --------------------------------------------------------

            int foodIndex = -1;
            double foodScore = double.MaxValue;
            double foodDistance = double.MaxValue;

            if (foods != null)
            {
                for (int i = 0; i < foods.Count; i++)
                {
                    Point3d food = foods[i];

                    if (!Finite(food.X) || !Finite(food.Y))
                    {
                        continue;
                    }

                    double d = Distance(me, food);

                    // 不鼓勵為了食物接近障礙物或角落
                    double score =
                        d + Exposure(food, world) * 2.0;

                    if (score < foodScore)
                    {
                        foodScore = score;
                        foodDistance = d;
                        foodIndex = i;
                    }
                }
            }

            // --------------------------------------------------------
            // 6. 搜尋五芒星
            // --------------------------------------------------------

            int starIndex = -1;
            double starScore = double.MaxValue;
            double starDistance = double.MaxValue;

            if (self.InvincibleSteps <= 0 &&
                self.Stars != null)
            {
                for (int i = 0; i < self.Stars.Count; i++)
                {
                    Point3d star = self.Stars[i];

                    if (!Finite(star.X) || !Finite(star.Y))
                    {
                        continue;
                    }

                    double d = Distance(me, star);

                    double score =
                        d + Exposure(star, world) * 2.0;

                    if (score < starScore)
                    {
                        starScore = score;
                        starDistance = d;
                        starIndex = i;
                    }
                }
            }

            // --------------------------------------------------------
            // 7. 建立朝向外部威脅的箭頭防禦隊形
            // --------------------------------------------------------

            // 先以群體平均移動方向作為箭頭方向
            Vector3d formationForward = alignment;

            // 有敵人時，讓箭頭尖端朝向敵人
            if (nearestEnemyIndex >= 0)
            {
                Point3d enemy =
                    neighbors[nearestEnemyIndex].Position;

                formationForward = Steering.Unit(
                    new Vector3d(
                        enemy.X - centerX,
                        enemy.Y - centerY,
                        0
                    )
                );
            }

            // 群體沒有明確方向時，先朝向安全的場地中心
            if (LengthSquared(formationForward) < EPS)
            {
                formationForward = Steering.Unit(
                    new Vector3d(
                        world.Center.X - centerX,
                        world.Center.Y - centerY,
                        0
                    )
                );
            }

            if (LengthSquared(formationForward) < EPS)
            {
                formationForward = forward;
            }

            if (LengthSquared(formationForward) < EPS)
            {
                formationForward = new Vector3d(1, 0, 0);
            }

            // 與箭頭方向垂直的側向向量
            Vector3d sideDirection = new Vector3d(
                -formationForward.Y,
                formationForward.X,
                0
            );

            // 每隻鳥相對於局部群體中心的位置
            Vector3d relativePosition = new Vector3d(
                me.X - centerX,
                me.Y - centerY,
                0
            );

            double frontPosition =
                Dot(relativePosition, formationForward);

            double sidePosition =
                Dot(relativePosition, sideDirection);

            // V 字形：側向越遠，理想的前進位置越靠後
            double idealFront =
                0.65 - Math.Abs(sidePosition) * 0.65;

            double formationError =
                Clamp(idealFront - frontPosition, -2.3, 2.3);

            Vector3d formation = Vector3d.Zero;

            if (friendCount >= 3)
            {
                formation =
                    formationForward * (formationError * 1.2);

                // 中央過度擁擠時，讓成員往兩翼微幅分散
                if (friendCount >= 6 &&
                    Math.Abs(sidePosition) < 0.35)
                {
                    double sign =
                        sidePosition < 0 ? -1.0 : 1.0;

                    formation +=
                        sideDirection * (sign * 0.4);
                }
            }

            // --------------------------------------------------------
            // 8. 整合主要行為
            // --------------------------------------------------------

            Vector3d desired = Vector3d.Zero;

            // 群聚與方向一致性
            if (friendCount >= 2)
            {
                desired +=
                    Steering.Unit(cohesion) *
                    (world.Phase == 3 ? 1.1 : 0.7);

                desired += alignment * 0.35;
            }

            // 防禦箭頭的幾何修正
            desired += formation;

            bool needEscape =
                LengthSquared(escape) > EPS;

            // 第一優先：避免被敵人從後方擊殺
            if (needEscape)
            {
                desired += Steering.Unit(escape) * 10.0;
            }
            else
            {
                // 第二優先：取得無敵能力
                bool chaseStar =
                    starIndex >= 0 &&
                    (foodIndex < 0 ||
                     starDistance <= foodDistance + 3.0);

                // 第三優先：搶奪食物繁殖
                // 決戰階段若敵人很近，允許突擊者優先進攻
                bool chaseFood =
                    foodIndex >= 0 &&
                    !(world.Phase == 3 &&
                      canAttack &&
                      bestTargetScore < 4.0);

                if (chaseStar)
                {
                    Point3d star = self.Stars[starIndex];

                    desired += Steering.Unit(
                        star - me
                    ) * 6.0;
                }
                else if (chaseFood)
                {
                    Point3d food = foods[foodIndex];

                    desired += Steering.Unit(
                        food - me
                    ) * 5.0;
                }
                else if (canAttack)
                {
                    // -----------------------------------------------
                    // 8-1. 攻擊者嘗試繞到敵人後方
                    // -----------------------------------------------

                    Neighbor target = neighbors[targetIndex];

                    Vector3d enemyVelocity = new Vector3d(
                        target.Velocity.X,
                        target.Velocity.Y,
                        0
                    );

                    if (!Finite(enemyVelocity.X) ||
                        !Finite(enemyVelocity.Y))
                    {
                        enemyVelocity = Vector3d.Zero;
                    }

                    Vector3d enemyForward =
                        Steering.Unit(enemyVelocity);

                    double d = Distance(me, target.Position);

                    // 預估敵人短時間後的位置
                    Point3d predicted = new Point3d(
                        target.Position.X +
                            enemyVelocity.X * 1.5,
                        target.Position.Y +
                            enemyVelocity.Y * 1.5,
                        0
                    );

                    // 先繞到敵人背後，避免直接正面對撞
                    Point3d attackPoint = new Point3d(
                        predicted.X - enemyForward.X * 1.2,
                        predicted.Y - enemyForward.Y * 1.2,
                        0
                    );

                    Vector3d enemyToMe = new Vector3d(
                        me.X - target.Position.X,
                        me.Y - target.Position.Y,
                        0
                    );

                    // 已在敵人後方且距離很近時，直接追擊
                    if (d < 1.5 &&
                        Dot(enemyForward, enemyToMe) < -0.25)
                    {
                        attackPoint = predicted;
                    }

                    desired += Steering.Unit(
                        attackPoint - me
                    ) * 5.5;
                }
                else
                {
                    // 非突擊者維持朝外警戒的防禦姿態
                    desired += formationForward * 0.3;

                    // 敵人太近時保持防禦距離
                    if (nearestEnemyIndex >= 0 &&
                        nearestEnemyDistance < 3.2 &&
                        self.InvincibleSteps <= 0)
                    {
                        Point3d enemy =
                            neighbors[nearestEnemyIndex].Position;

                        Vector3d away = Steering.Unit(
                            me - enemy
                        );

                        desired += away *
                            (3.2 - nearestEnemyDistance) * 1.4;
                    }
                }
            }

            // --------------------------------------------------------
            // 9. 避免接近場地邊界與障礙物
            // --------------------------------------------------------

            Vector3d wallAvoidance =
                AvoidWalls(me, velocity, world);

            Vector3d obstacleAvoidance =
                AvoidObstacles(me, velocity, world);

            // 縮小階段提高邊界迴避權重
            double wallWeight =
                world.Phase == 2 ? 11.0 : 8.0;

            desired += wallAvoidance * wallWeight;
            desired += obstacleAvoidance * 11.0;

            // --------------------------------------------------------
            // 10. 避免靜止與無效向量
            // --------------------------------------------------------

            desired.Z = 0;

            if (!Finite(desired.X) ||
                !Finite(desired.Y))
            {
                return Vector3d.Zero;
            }

            if (LengthSquared(desired) < EPS)
            {
                // 沒有明確目標時先保持原來的移動方向
                desired = velocity;

                // 完全靜止時使用自己的固定種子亂數來源
                if (LengthSquared(desired) < EPS)
                {
                    double angle = 0.0;

                    if (self.Rng != null)
                    {
                        angle =
                            self.Rng.NextDouble() *
                            Math.PI * 2.0;
                    }

                    desired = new Vector3d(
                        Math.Cos(angle),
                        Math.Sin(angle),
                        0
                    );
                }
            }

            // --------------------------------------------------------
            // 11. 轉換成引擎可接受的轉向力
            // --------------------------------------------------------

            Vector3d result =
                Steering.Seek(desired, velocity, world);

            // 最後再次檢查，避免無效數值淘汰整隊
            if (!Finite(result.X) ||
                !Finite(result.Y))
            {
                return Vector3d.Zero;
            }

            return new Vector3d(
                result.X,
                result.Y,
                0
            );
        }

        // ============================================================
        // 12. 輔助函式：向量內積
        // ============================================================

        private static double Dot(Vector3d a, Vector3d b)
        {
            return a.X * b.X + a.Y * b.Y;
        }

        // ============================================================
        // 13. 輔助函式：向量長度平方
        // ============================================================

        private static double LengthSquared(Vector3d v)
        {
            return v.X * v.X + v.Y * v.Y;
        }

        // ============================================================
        // 14. 輔助函式：平面兩點距離
        // ============================================================

        private static double Distance(Point3d a, Point3d b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;

            return Math.Sqrt(dx * dx + dy * dy);
        }

        // ============================================================
        // 15. 輔助函式：限制數值範圍
        // ============================================================

        private static double Clamp(
            double value,
            double minimum,
            double maximum)
        {
            return Math.Max(minimum, Math.Min(maximum, value));
        }

        // ============================================================
        // 16. 輔助函式：檢查數值是否有效
        // ============================================================

        private static bool Finite(double value)
        {
            return !double.IsNaN(value) &&
                   !double.IsInfinity(value);
        }

        // ============================================================
        // 17. 輔助函式：判斷目標位置是否危險
        // ============================================================

        private static double Exposure(
            Point3d point,
            World world)
        {
            double risk = 0.0;

            // 計算距離四個邊界的剩餘空間
            double freeX =
                world.HalfSize -
                Math.Abs(point.X - world.Center.X);

            double freeY =
                world.HalfSize -
                Math.Abs(point.Y - world.Center.Y);

            // 目標靠近邊界時提高風險
            if (freeX < 1.6)
            {
                risk += (1.6 - freeX) * 2.0;
            }

            if (freeY < 1.6)
            {
                risk += (1.6 - freeY) * 2.0;
            }

            // 同時靠近兩個邊界代表接近角落
            if (freeX < 2.0 && freeY < 2.0)
            {
                risk += 3.0;
            }

            // 計算障礙物附近的風險
            if (world.Obstacles != null)
            {
                foreach (var obstacle in world.Obstacles)
                {
                    double d = obstacle.DistanceTo(point);

                    if (!Finite(d))
                    {
                        continue;
                    }

                    if (d < 1.4)
                    {
                        risk += (1.4 - d) * 2.0;
                    }
                }
            }

            return Math.Min(risk, 20.0);
        }

        // ============================================================
        // 18. 輔助函式：場地邊界預測與迴避++
        // ============================================================

        private static Vector3d AvoidWalls(
            Point3d position,
            Vector3d velocity,
            World world)
        {
            // 預測未來四步的移動位置
            double futureX =
                position.X + velocity.X * 4.0;

            double futureY =
                position.Y + velocity.Y * 4.0;

            double h = world.HalfSize;

            double left = world.Center.X - h;
            double right = world.Center.X + h;
            double bottom = world.Center.Y - h;
            double top = world.Center.Y + h;

            // 隨場地縮小，自動縮小避讓緩衝區
            double margin = Math.Min(
                2.3,
                Math.Max(0.35, h * 0.42)
            );

            double forceX = 0.0;
            double forceY = 0.0;

            if (futureX - left < margin)
            {
                forceX +=
                    (margin - (futureX - left)) / margin;
            }

            if (right - futureX < margin)
            {
                forceX -=
                    (margin - (right - futureX)) / margin;
            }

            if (futureY - bottom < margin)
            {
                forceY +=
                    (margin - (futureY - bottom)) / margin;
            }

            if (top - futureY < margin)
            {
                forceY -=
                    (margin - (top - futureY)) / margin;
            }

            return new Vector3d(forceX, forceY, 0);
        }

        // ============================================================
        // 19. 輔助函式：矩形障礙物迴避
        // ============================================================

        private static Vector3d AvoidObstacles(
            Point3d position,
            Vector3d velocity,
            World world)
        {
            if (world.Obstacles == null)
            {
                return Vector3d.Zero;
            }

            // 預測短時間後的位置
            Point3d future = new Point3d(
                position.X + velocity.X * 3.5,
                position.Y + velocity.Y * 3.5,
                0
            );

            double buffer = 1.3;

            Vector3d avoidance = Vector3d.Zero;

            foreach (var obstacle in world.Obstacles)
            {
                double distance =
                    obstacle.DistanceTo(future);

                if (!Finite(distance) || distance >= buffer)
                {
                    continue;
                }

                Point3d closest =
                    obstacle.ClosestPoint(future);

                Vector3d away = new Vector3d(
                    future.X - closest.X,
                    future.Y - closest.Y,
                    0
                );

                // 已在障礙物內部或邊界上時，
                // 改為沿最短方向離開矩形
                if (distance <= EPS)
                {
                    double rx =
                        future.X - obstacle.Center.X;

                    double ry =
                        future.Y - obstacle.Center.Y;

                    double remainingX =
                        obstacle.HalfWidth - Math.Abs(rx);

                    double remainingY =
                        obstacle.HalfHeight - Math.Abs(ry);

                    if (remainingX < remainingY)
                    {
                        away = new Vector3d(
                            rx >= 0 ? 1 : -1,
                            0,
                            0
                        );
                    }
                    else
                    {
                        away = new Vector3d(
                            0,
                            ry >= 0 ? 1 : -1,
                            0
                        );
                    }
                }

                double strength =
                    Clamp((buffer - distance) / buffer, 0, 1);

                avoidance +=
                    Steering.Unit(away) * strength;
            }

            return avoidance;
        }
    }
}
