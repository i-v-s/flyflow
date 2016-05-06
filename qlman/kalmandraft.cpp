#include "kalmandraft.h"
#include "../normdist.h"

KalmanDraft::KalmanDraft(QObject *parent) : QObject(parent),
   a(0)
{
    pos << 0, 0;
    vel << 1, 1;
    fts[0] << 20, 30;
    fts[1] << -30, 20;
}

void KalmanDraft::step(double dt)
{
    pos += vel * dt;
    k.predict();
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
