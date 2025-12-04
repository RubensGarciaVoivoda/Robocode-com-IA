package meusrobos; // Se der erro de package, APAGUE esta linha.

import robocode.*;
import robocode.util.Utils;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * SuperSpinBot com IA adaptativa + KNN
 * - Mantém movimento circular original
 * - Mira com 3 "armas": Head-On, Linear e Circular com aprendizado de velocidade
 * - Escolhe automaticamente a arma que mais acerta (global) e,
 *   quando tem dados suficientes, usa KNN para escolher a melhor arma
 *   para a situação atual (distância + velocidade lateral + velocidade do inimigo).
 */

public class SuperSpinBot extends AdvancedRobot {

    // ---- VARIÁVEIS DO GUN ORIGINAL (para aprender velocidade do inimigo) ----
    static double enemyVelocities[][] = new double[400][4];
    static int currentEnemyVelocity;
    static int aimingEnemyVelocity;
    double velocityToAimAt;
    boolean fired;
    double oldTime;
    int count;
    int averageCount;

    // ---- IA: TIPOS DE ARMA ----
    private static final int GUN_HEAD_ON = 0;
    private static final int GUN_LINEAR = 1;
    private static final int GUN_CIRCULAR = 2;

    // gunStats[gunType][0] = tiros disparados, gunStats[gunType][1] = tiros acertados
    private double[][] gunStats = new double[3][2];

    // ---- KNN: samples de tiros ----
    static class ShotSample {
        double distance;
        double lateralVelocity;
        double enemyVelocity;
        int gunType;
        boolean resultKnown;
        boolean hit;

        ShotSample(double distance, double lateralVelocity, double enemyVelocity, int gunType) {
            this.distance = distance;
            this.lateralVelocity = lateralVelocity;
            this.enemyVelocity = enemyVelocity;
            this.gunType = gunType;
            this.resultKnown = false;
            this.hit = false;
        }
    }

    private ArrayList<ShotSample> samples = new ArrayList<ShotSample>();
    private Map<Bullet, Integer> bulletSampleIndex = new HashMap<Bullet, Integer>();
    private static final int MAX_SAMPLES = 800;

    // Mapa para saber qual arma disparou cada bala (pra estatística global)
    private Map<Bullet, Integer> bulletGunMap = new HashMap<Bullet, Integer>();

    // ---- MOVIMENTO ORIGINAL DO SUPERSPINBOT ----
    static double turn = 2;
    int turnDir = 1;
    int moveDir = 1;
    double oldEnemyHeading;
    double oldEnergy = 100;

    @Override
    public void run() {
        // Cores
        setBodyColor(Color.blue);
        setGunColor(Color.blue);
        setRadarColor(Color.black);
        setScanColor(Color.yellow);

        // Desacopla radar e canhão do corpo
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        while (true) {
            turnRadarRightRadians(Double.POSITIVE_INFINITY);
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
        double absBearing = e.getBearingRadians() + getHeadingRadians();

        // ====================== MOVIMENTO (ORIGINAL) ======================

        // aumenta velocidade de giro de forma pseudo-aleatória entre 2 e 8
        turn += 0.2 * Math.random();
        if (turn > 8) {
            turn = 2;
        }

        // quando o inimigo atira (queda de energia), mudamos direção e às vezes frente/trás
        double energyDrop = oldEnergy - e.getEnergy();
        if (energyDrop <= 3 && energyDrop >= 0.1) {
            if (Math.random() > 0.5) {
                turnDir *= -1;
            }
            if (Math.random() > 0.8) {
                moveDir *= -1;
            }
        }

        // velocidade máxima reduz conforme aumenta o turn rate
        setMaxTurnRate(turn);
        setMaxVelocity(12 - turn);
        setAhead(90 * moveDir);
        setTurnLeft(90 * turnDir);
        oldEnergy = e.getEnergy();

        // ====================== APRENDIZADO DA VELOCIDADE (ORIGINAL) ======================

        // segmenta a velocidade do inimigo
        double enemyVel = e.getVelocity();
        if (enemyVel < -2) {
            currentEnemyVelocity = 0;
        } else if (enemyVel > 2) {
            currentEnemyVelocity = 1;
        } else { // área central
            if (currentEnemyVelocity == 0) {
                currentEnemyVelocity = 2;
            } else if (currentEnemyVelocity == 1) {
                currentEnemyVelocity = 3;
            }
        }

        // atualiza o segmento quando o tiro teve tempo de chegar
        if (getTime() - oldTime > e.getDistance() / 12.8 && fired) {
            aimingEnemyVelocity = currentEnemyVelocity;
        } else {
            fired = false;
        }

        // grava nova velocidade na tabela circular
        enemyVelocities[count][aimingEnemyVelocity] = enemyVel;
        count++;
        if (count == 400) {
            count = 0;
        }

        // calcula velocidade média para o segmento atual
        averageCount = 0;
        velocityToAimAt = 0;
        while (averageCount < 400) {
            velocityToAimAt += enemyVelocities[averageCount][currentEnemyVelocity];
            averageCount++;
        }
        velocityToAimAt /= 400.0;

        // ====================== MIRA COM IA (3 ARMAS) ======================

        double bulletPower = Math.min(2.4, Math.min(e.getEnergy() / 4, getEnergy() / 10));

        double myX = getX();
        double myY = getY();
        double enemyX = myX + e.getDistance() * Math.sin(absBearing);
        double enemyY = myY + e.getDistance() * Math.cos(absBearing);
        double enemyHeading = e.getHeadingRadians();
        double enemyHeadingChange = enemyHeading - oldEnemyHeading;
        oldEnemyHeading = enemyHeading;

        // velocidade lateral (feature pro KNN)
        double lateralVelocity = enemyVel * Math.sin(enemyHeading - absBearing);

        // Arma 0: Head-On (atira onde o inimigo está agora)
        double angleHeadOn = absBearing;

        // Arma 1: Linear targeting
        double angleLinear = calcLinearAngle(
                myX, myY,
                enemyX, enemyY,
                enemyHeading,
                enemyVel,
                bulletPower
        );

        // Arma 2: Circular com aprendizado de velocidade (usando velocityToAimAt)
        double angleCircular;
        {
            double deltaTime = 0;
            double battleFieldHeight = getBattleFieldHeight();
            double battleFieldWidth = getBattleFieldWidth();
            double predictedX = enemyX;
            double predictedY = enemyY;
            double bulletSpeed = 20.0 - 3.0 * bulletPower;

            while ((++deltaTime) * bulletSpeed <
                    Point2D.distance(myX, myY, predictedX, predictedY)) {

                // usa velocidade média aprendida para esse segmento
                predictedX += Math.sin(enemyHeading) * velocityToAimAt;
                predictedY += Math.cos(enemyHeading) * velocityToAimAt;
                enemyHeading += enemyHeadingChange;

                // se bater na parede, para a simulação
                if (predictedX < 18.0
                        || predictedY < 18.0
                        || predictedX > battleFieldWidth - 18.0
                        || predictedY > battleFieldHeight - 18.0) {

                    predictedX = Math.min(Math.max(18.0, predictedX),
                            battleFieldWidth - 18.0);
                    predictedY = Math.min(Math.max(18.0, predictedY),
                            battleFieldHeight - 18.0);
                    break;
                }
            }

            angleCircular = Utils.normalAbsoluteAngle(Math.atan2(
                    predictedX - myX, predictedY - myY));
        }

        // ====================== ESCOLHA DA MELHOR ARMA (KNN + GLOBAL) ======================

        // tenta usar KNN, se ainda não tiver dados suficientes, usa global
        int bestGun = selectBestGunTypeKNN(e.getDistance(), lateralVelocity, enemyVel);

        double angleToUse;
        if (bestGun == GUN_HEAD_ON) {
            angleToUse = angleHeadOn;
        } else if (bestGun == GUN_LINEAR) {
            angleToUse = angleLinear;
        } else {
            angleToUse = angleCircular;
        }

        // trava radar no inimigo
        setTurnRadarRightRadians(Utils.normalRelativeAngle(
                absBearing - getRadarHeadingRadians()) * 2);

        // mira canhão no ângulo escolhido
        setTurnGunRightRadians(Utils.normalRelativeAngle(
                angleToUse - getGunHeadingRadians()));

        // dispara quando possível
        if (getGunHeat() == 0) {
            Bullet b = setFireBullet(bulletPower);
            if (b != null) {
                fired = true;
                oldTime = getTime();

                // registra tiro globalmente
                gunStats[bestGun][0]++; // tiros++

                // cria sample para o KNN
                ShotSample sample = new ShotSample(e.getDistance(), lateralVelocity, enemyVel, bestGun);
                samples.add(sample);
                int index = samples.size() - 1;

                bulletSampleIndex.put(b, index);
                bulletGunMap.put(b, bestGun);

                // controla tamanho da memória
                if (samples.size() > MAX_SAMPLES) {
                    samples.remove(0);
                    // não ajustamos índices antigos no mapa por simplicidade.
                    // em batalhas casuais isso não é crítico.
                }
            }
        }
    }

    // ========= IA: ESCOLHA DA ARMA PELO DESEMPENHO GLOBAL =========

    private int selectBestGunType() {
        int best = GUN_HEAD_ON;
        double bestScore = -1;

        for (int i = 0; i < 3; i++) {
            double shots = gunStats[i][0];
            double hits = gunStats[i][1];

            // suavização: (hits + 1) / (shots + 2) para evitar matar arma nova
            double score = (hits + 1.0) / (shots + 2.0);
            if (score > bestScore) {
                bestScore = score;
                best = i;
            }
        }
        return best;
    }

    // ========= IA: ESCOLHA DA ARMA COM KNN =========

    private int selectBestGunTypeKNN(double distance, double latVel, double enemyVel) {
        int K = 15;                 // número de vizinhos
        int MIN_SAMPLES = 20;       // mínimo de samples pra usar KNN

        if (samples.size() < MIN_SAMPLES) {
            // poucos dados ainda, usa heurística global simples
            return selectBestGunType();
        }

        double bestScore = -1;
        int bestGun = selectBestGunType(); // fallback

        for (int gunType = 0; gunType < 3; gunType++) {
            double score = knnScoreForGun(gunType, distance, latVel, enemyVel, K);
            if (score > bestScore) {
                bestScore = score;
                bestGun = gunType;
            }
        }

        return bestGun;
    }

    private double knnScoreForGun(int gunType, double distance, double latVel, double enemyVel, int K) {
        double[] bestDist = new double[K];
        boolean[] bestHit = new boolean[K];

        // inicializa
        for (int i = 0; i < K; i++) {
            bestDist[i] = Double.POSITIVE_INFINITY;
            bestHit[i] = false;
        }

        // percorre samples
        for (int i = 0; i < samples.size(); i++) {
            ShotSample s = samples.get(i);
            if (s.gunType != gunType || !s.resultKnown) {
                continue;
            }

            double dDist = s.distance - distance;
            double dLat = s.lateralVelocity - latVel;
            double dVel = s.enemyVelocity - enemyVel;

            // métrica simples (pode ajustar pesos)
            double dist2 = dDist * dDist + (dLat * dLat) / 4.0 + (dVel * dVel) / 4.0;

            // insere entre os K melhores, se for o caso
            for (int k = 0; k < K; k++) {
                if (dist2 < bestDist[k]) {
                    for (int m = K - 1; m > k; m--) {
                        bestDist[m] = bestDist[m - 1];
                        bestHit[m] = bestHit[m - 1];
                    }
                    bestDist[k] = dist2;
                    bestHit[k] = s.hit;
                    break;
                }
            }
        }

        // se não achou nenhum vizinho, score ruim
        if (bestDist[0] == Double.POSITIVE_INFINITY) {
            return -1;
        }

        double weightedHits = 0;
        double weightedTotal = 0;

        for (int i = 0; i < K; i++) {
            if (bestDist[i] == Double.POSITIVE_INFINITY) continue;

            double weight = 1.0 / (1.0 + bestDist[i]); // vizinhos mais próximos valem mais
            weightedTotal += weight;
            if (bestHit[i]) {
                weightedHits += weight;
            }
        }

        if (weightedTotal == 0) return -1;

        return weightedHits / weightedTotal; // probabilidade estimada de acerto
    }

    // ========= MIRA LINEAR =========

    private double calcLinearAngle(double myX, double myY,
                                   double enemyX, double enemyY,
                                   double enemyHeading, double enemyVelocity,
                                   double bulletPower) {

        double futureX = enemyX;
        double futureY = enemyY;
        double bulletSpeed = 20.0 - 3.0 * bulletPower;
        double deltaTime = 0;

        double battleFieldWidth = getBattleFieldWidth();
        double battleFieldHeight = getBattleFieldHeight();

        while ((++deltaTime) * bulletSpeed <
                Point2D.distance(myX, myY, futureX, futureY) &&
                deltaTime < 40) {

            futureX += Math.sin(enemyHeading) * enemyVelocity;
            futureY += Math.cos(enemyHeading) * enemyVelocity;

            // mantém dentro do campo
            if (futureX < 18.0 || futureX > battleFieldWidth - 18.0) {
                futureX = Math.min(Math.max(18.0, futureX), battleFieldWidth - 18.0);
                break;
            }
            if (futureY < 18.0 || futureY > battleFieldHeight - 18.0) {
                futureY = Math.min(Math.max(18.0, futureY), battleFieldHeight - 18.0);
                break;
            }
        }

        return Utils.normalAbsoluteAngle(Math.atan2(
                futureX - myX, futureY - myY));
    }

    // ========= EVENTOS DE TIRO: ATUALIZA ESTATÍSTICAS =========

    @Override
    public void onBulletHit(BulletHitEvent event) {
        Bullet b = event.getBullet();

        // atualiza gunStats global
        Integer gunType = bulletGunMap.get(b);
        if (gunType != null) {
            gunStats[gunType][1]++; // acertos++
            bulletGunMap.remove(b);
        }

        // atualiza sample do KNN
        Integer idx = bulletSampleIndex.get(b);
        if (idx != null) {
            int i = idx.intValue();
            if (i >= 0 && i < samples.size()) {
                ShotSample s = samples.get(i);
                s.resultKnown = true;
                s.hit = true;
            }
            bulletSampleIndex.remove(b);
        }
    }

    @Override
    public void onBulletMissed(BulletMissedEvent event) {
        Bullet b = event.getBullet();

        // não precisa mexer em gunStats aqui, o erro já está no shots

        // atualiza sample do KNN
        Integer idx = bulletSampleIndex.get(b);
        if (idx != null) {
            int i = idx.intValue();
            if (i >= 0 && i < samples.size()) {
                ShotSample s = samples.get(i);
                s.resultKnown = true;
                s.hit = false;
            }
            bulletSampleIndex.remove(b);
        }
    }
}
