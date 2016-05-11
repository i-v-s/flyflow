#include "kalmandraft.h"
#include "../normdist.h"

KalmanDraft::KalmanDraft(QObject *parent) : QObject(parent),
   a(0)
{
    pos << 0, 0;
    vel << 0.3, 2;
    k.X_(0) = vel(0);
    k.X_(1) = vel(1);
    fts[0] << 40, 30;
    fts[1] << 80, 20;
    fts[2] << 60, 50;
    fts[3] << 50, 100;
}

void KalmanDraft::step(double dt)
{
    pos += vel * dt;
    k.predict();
    int n = sizeof(fts) / sizeof(*fts);
    Eigen::Matrix<double, 4, 1> z;
    for(int x = 0; x < n; x++)
    {
        Eigen::Vector2d fp = fts[x] - pos;
        z(x) = atan2(fp(1), fp(0));// + a;
    }
    k.correct(z);
    for(int n = 4; n < k.X_.rows(); n += 2)
    {
        int N = (n - 4) / 2;
        Eigen::Matrix2d s = k.Sigma_.block<2, 2>(n, n);
        Eigen::Vector2d m = k.X_.block<2, 1>(n, 0);
        NormalDistribution<double, 2> nd(m, s);
        NDEllipse e = nd.ellipse(2);
        QVariant x(e.cx), y(e.cy), a(e.alpha), radx(e.rx), rady(e.ry);
        QVariant rx(fts[N](0)), ry(fts[N](1));
        emit onFeature(QVariant(N), x, y, a, radx, rady, rx, ry);
    }
    QVariant tx(pos(0)), ty(pos(1)), ta(a);
    emit onPos(tx, ty, ta);
}
