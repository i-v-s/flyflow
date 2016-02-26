class GN
{
public:
    cv::Mat e_, a_, b_; // error, matrix A, vector B, Jacobian
    cv::Mat gx_, gy_; // Gradients
    cv::Mat gxx_, gxy_, gyx_, gyy_; // Gradients * (X или Y)
    cv::Mat t_; // Affine transformation matrix
    int level_;
    cv::Mat * j_[6];
    GN():
        j_{
             &gxx_, &gxy_, &gx_,
             &gyx_, &gyy_, &gy_
         }
    {}

    inline void reset() { t_ = cv::Mat::eye(2, 3, CV_64F); level_ = 0; }
    inline void scaleBy(double s)
    {
        t_.at<double>(0, 2) *= s;
        t_.at<double>(1, 2) *= s;
    }

    template<class T1, class T2 = T1> static inline double scalMul(const cv::Mat & m1, const cv::Mat & m2)
    {
        assert(m1.rows == m2.rows && m2.cols == m2.cols);
        double sum = 0;
        const T1 * a = (const T1 *) m1.data;
        const T2 * b = (const T2 *) m2.data;
        for(const T1 * end = (const T1 *)m1.dataend; a < end; a++, b++)
            sum += double(*a) * double(*b);
        return sum;
    }
    template<class Ts, class Td> static void mulByX(const cv::Mat & src, cv::Mat & dst)
    {
        assert(src.rows == dst.rows && src.cols == dst.cols);
        const Ts * s = (const Ts *) src.data;
        Td * d = (Td *) dst.data;
        for(int h = src.rows, w = src.cols; h--; )
            for(int x = 0; x < w; x++)
                *(d++) = (*(s++) * x);// / ((w >> 4) + 1);
    }
    template<class Ts, class Td> static void mulByY(const cv::Mat & src, cv::Mat & dst)
    {
        assert(src.rows == dst.rows && src.cols == dst.cols);
        const Ts * s = (const Ts *) src.data;
        Td * d = (Td *) dst.data;
        for(int y = 0, h = src.rows, w = src.cols; y < h; y++)
            for(int x = w; x--; )
                *(d++) = (*(s++) * y);// / ((h >> 4) + 1);
    }

    double calcError(const cv::Mat & f0, const cv::Mat & f1, const cv::Mat & mask)
    {
        // Calculate error
        cv::subtract(f1, f0, e_, cv::noArray(), CV_16S);
        cv::multiply(e_, mask, e_, 1, CV_16S);
        return cv::norm(e_);
    }

    cv::Mat calcShiftStep(const cv::Mat & i1, const cv::Mat & i2, const cv::Mat & mask)
    {
        const int ks = 32, ks2 = ks * ks;
        cv::Sobel(i2, gx_, CV_16S, 1, 0, CV_SCHARR);
        cv::Sobel(i2, gy_, CV_16S, 0, 1, CV_SCHARR);
        a_ = cv::Mat(2, 2, CV_64F);
        a_.at<double>(0, 0) = scalMul<int16_t>(gx_, gx_) / ks2;
        a_.at<double>(1, 1) = scalMul<int16_t>(gy_, gy_) / ks2;
        a_.at<double>(0, 1) = a_.at<double>(1, 0) = scalMul<int16_t>(gx_, gy_) / ks2;

        cv::subtract(i2, i1, e_, cv::noArray(), CV_16S);
        cv::multiply(e_, mask, e_, 1, CV_16S);

        b_ = cv::Mat(2, 1, CV_64F);
        b_.at<double>(0, 0) = scalMul<int16_t>(gx_, e_) / (100 * ks);
        b_.at<double>(1, 0) = scalMul<int16_t>(gy_, e_) / (100 * ks);

        cv::Mat du;
        cv::solve(a_, b_, du);
        cv::Mat dt = cv::Mat::eye(2, 3, CV_64F);
        dt.at<double>(0, 2) = du.at<double>(0, 0);
        dt.at<double>(1, 2) = du.at<double>(1, 0);
        return dt;
    }
    void calcAffineJacobian(const cv::Mat & f1)
    {
        int w = f1.cols, h = f1.rows;
        //int wm = (int) sqrt(w), hm = (int) sqrt(h);
        int wm = 1, hm = 1;
        /*if(level_ == 3)
        {
            wm = 1000;
            hm = 1000;
        }*/
        // Calculate gradients
        const double ks = 1.0 / 32, ks2 = ks * ks;
        cv::Mat gx, gy;
        cv::Sobel(f1, gx, CV_16S, 1, 0, CV_SCHARR);
        cv::Sobel(f1, gy, CV_16S, 0, 1, CV_SCHARR);
        gx.convertTo(gx_, CV_32S);
        gy.convertTo(gy_, CV_32S);

        // Calculate gradients multiplied by pos
        gxx_ = cv::Mat(h, w, CV_32S);
        gxy_ = cv::Mat(h, w, CV_32S);
        gyx_ = cv::Mat(h, w, CV_32S);
        gyy_ = cv::Mat(h, w, CV_32S);
        mulByX<int32_t, int32_t>(gx_, gxx_);
        mulByY<int32_t, int32_t>(gx_, gxy_);
        mulByX<int32_t, int32_t>(gy_, gyx_);
        mulByY<int32_t, int32_t>(gy_, gyy_);
        gx_ *= wm;
        gy_ *= hm;

        // Calculate matrix A = j ^ 2
        a_ = cv::Mat(6, 6, CV_64F);
        for(int x = 0; x < 6; x++) for(int y = 0; y < 6; y++)
            a_.at<double>(y, x) = scalMul<int32_t>(*j_[x], *j_[y]) * ks2;


    }

    cv::Mat calcAffineStep()
    {
        int wm = 1, hm = 1;
        const double ks = 1.0 / 32;
        // Calculate vector B
        b_ = cv::Mat(6, 1, CV_64F);
        for(int y = 0; y < 6; y++)
            b_.at<double>(y, 0) = scalMul<int32_t, int16_t>(*j_[y], e_) * (ks / 100);

        // Calculate step
        cv::Mat dt;
        cv::solve(a_, b_, dt);
        dt.rows = 2;
        dt.cols = 3;
        dt.at<double>(0, 2) *= wm;
        dt.at<double>(1, 2) *= hm;
        return dt;
    }

    inline void test1()
    {
        int w = 20, h = 20;
        cv::Mat a(h, w, CV_8U), b;
        uint8_t * p = a.data;
        for(int y = 0; y < h; y++) for(int x = 0; x < w; x++)
            *(p++) = x * (w - x) + 2 * y;
        cv::Mat t = cv::Mat::eye(2, 3, CV_64F);
        t.at<double>(0, 2) = 2;

        for(int x = 0; x < 10; x++)
        {
            cv::warpAffine(a, b, t, cv::Size(h, w));
            cv::Mat du = calcShiftStep(a, b, a);
            cout << "!!! iteration " << x + 1 << endl;
            //cout << "gx = " << endl << gx_ << endl << endl;
            //cout << "gy = " << endl << gy_ << endl << endl;
            cout << "t = " << endl << t << endl << endl;
            cout << "a = " << endl << a_ << endl << endl;
            cout << "b = " << endl << b_ << endl << endl;
            cout << "du = " << endl << du << endl << endl;
            t.at<double>(0, 2) += du.at<double>(0, 0);
            t.at<double>(1, 2) += du.at<double>(1, 0);
        }
    }
    inline void test2()
    {
        int w = 10, h = 10;
        cv::Mat a(h, w, CV_8U), b;
        uint8_t * p = a.data;
        for(int y = 0; y < h; y++) for(int x = 0; x < w; x++)
            *(p++) = x * (w - x) + 2 * y;
        cv::Mat t = cv::Mat::eye(2, 3, CV_64F);
        t.at<double>(0, 2) = 2;

        for(int x = 0; x < 3; x++)
        {
            cv::warpAffine(a, b, t, cv::Size(h, w));
            cv::Mat du = calcAffineStep();
            cout << "!!! iteration " << x + 1 << endl;
            cout << "gx = " << endl << gx_ << endl << endl;
            cout << "gy = " << endl << gy_ << endl << endl;
            cout << "t = " << endl << t << endl << endl;
            cout << "e = " << cv::norm(e_) << endl;
            cout << "a = " << endl << a_ << endl << endl;
            cout << "b = " << endl << b_ << endl << endl;
            cout << "du = " << endl << du << endl << endl;

            t += du;
        }
    }
    bool demo(const cv::Mat & a, const cv::Mat & b, cv::Mat & out)
    {
        std::vector<cv::Mat> v = {a, b, b};
        cv::merge(v, out);

        int h = a.rows, w = a.cols;
        cv::Mat whiteBox(a.rows, a.cols, CV_8U, cv::Scalar(100)), mask;
        double e = 1E10;
        cv::Mat optT = t_.clone();
        double step = 1.0;
        calcAffineJacobian(b);
        for(int x = 0; x < 20; x++)
        {
            /*cv::Mat bt;
            cv::warpAffine(b, bt, t_, cv::Size(w, h), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(255));
            cv::warpAffine(whiteBox, mask, t_, cv::Size(w, h));

            double te = calcError(a, bt, mask);
            cv::Mat du = calcAffineStep(bt, mask);*/


            cv::Mat at;
            cv::warpAffine(a, at, t_, cv::Size(w, h), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar(255));
            cv::warpAffine(whiteBox, mask, t_, cv::Size(w, h), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);

            double te = calcError(at, b, mask);
            cv::Mat du = calcAffineStep();


            if(te >= e)
            {
                t_ = optT.clone();
                step *= 0.3;
                if(step > 0.05) continue;
                if(e / e_.total() > 150)
                {
                    cv::vconcat(out, cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 255)), out);
                    return false;
                }
                return true;
            }
            std::vector<cv::Mat> v = {at, b, b};
            cv::merge(v, at);
            //if(!x) bt.copyTo(out);
            cv::vconcat(out, at, out);

            optT = t_.clone();
            e = te;
            t_ += du * step;
        }
        return true;
    }
};


/*
std::unique_ptr<Frame> prev;
cv::Mat map;
cv::Point pos(0, 0);
int pyrct = 7;
bool updPrev = true;
bool pause = false;
GN gnn;
//gnn.test2();
for(;;)
{
    cv::Mat frame, res;
    cap >> frame; // get a new frame from camera
    //cv::cvtColor(frame, res, CV_BGR2GRAY);

    std::unique_ptr<Frame> f(new Frame(frame, pyrct));
    //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
    //Canny(edges, edges, 0, 30, 3);
    std::vector<cv::Mat> trs;
    if(prev != nullptr && !pause)
    {
        int lvl =  5;
        const cv::Mat & i1 = prev->pyramid[lvl];
        const cv::Mat & i2 = f->pyramid[lvl];
        cv::Mat r;
        std::vector<cv::Mat> v = {i1, i2, i2};
        cv::merge(v, r);
        cv::Mat out, all;
        gnn.reset();

        cout << "size" << lvl << " = "<< f->pyramid[lvl].size << endl;
        cout << "Tb" << lvl << " = " << endl << gnn.t_ << endl << endl;
        gnn.level_ = lvl;
        while(gnn.demo(prev->pyramid[lvl], f->pyramid[lvl], out) && lvl > 1)
        {
            if(all.empty()) all = out;
            else
            {
                if(out.rows > all.rows) cv::vconcat(all, cv::Mat(out.rows - all.rows, all.cols, CV_8UC3, cv::Scalar(0, 0, 0)), all);
                else if(all.rows > out.rows) cv::vconcat(out, cv::Mat(all.rows - out.rows, out.cols, CV_8UC3, cv::Scalar(0, 0, 0)), out);
                cv::hconcat(all, out, all);
            }
            cout << "Te" << lvl << " = " << endl << gnn.t_ << endl << endl;
            lvl--;
            gnn.level_ = lvl;
            gnn.scaleBy(2);
            cout << "Tb" << lvl << " = " << endl << gnn.t_ << endl << endl;
        }
        if(all.empty()) all = out;
        cv::resize(all, all, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
        cv::imshow("all", all);

    }
    char k = cv::waitKey(1);
    if(k == 'i')
        updPrev = !updPrev;
    if(k == 'p')
    {
        pause = !pause;
    }
    if(k == 'q') break;
    if(updPrev)
    {
        prev = std::move(f);
    }
}*/
