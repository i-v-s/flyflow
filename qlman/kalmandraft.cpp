#include "kalmandraft.h"

KalmanDraft::KalmanDraft(QObject *parent) : QObject(parent)
{

}

void KalmanDraft::step(double dt)
{
    k.predict();
    for(int n = 4; n < k.X_.rows(); n += 2)
    {
        QVariant x(k.X_(n)), y(k.X_(n + 1)), sxx(k.Sigma_(n, n)), sxy(k.Sigma_(n, n + 1)), syy(k.Sigma_(n + 1, n + 1));
        emit onFeature(QVariant((n - 4) / 2), x, y, sxx, sxy, syy);
    }
}
