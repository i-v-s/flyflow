#include "kalmandraft.h"

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
        QVariant x(k.X_(n)), y(k.X_(n + 1)),
                sxx(k.Sigma_(n, n)),
                sxy(k.Sigma_(n, n + 1)),
                syy(k.Sigma_(n + 1, n + 1));
        QVariant rx(fts[N](0)), ry(fts[N](1));
        emit onFeature(QVariant(N), x, y, sxx, sxy, syy, rx, ry);
    }
    QVariant tx(pos(0)), ty(pos(1)), ta(a);
    emit onPos(tx, ty, ta);
}
