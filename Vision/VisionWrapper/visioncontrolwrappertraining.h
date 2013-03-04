#ifndef CONTROLWRAPPERTRAINING_H
#define CONTROLWRAPPERTRAINING_H

#include "Vision/visioncontroller.h"
#include "Vision/VisionWrapper/datawrappertraining.h"

class NUSensorsData;
class NUActionatorsData;

class VisionControlWrapper
{
public:
    static VisionControlWrapper* getInstance();

    int runFrame();
    int runFrame(NUImage& img);
    bool setLUT(const string& filename);
    bool setImageStream(const string& filename);
    void restartStream();
    void resetHistory();
    bool renderFrame(cv::Mat& mat, bool lines_only=false);
    void writeBatchDetections(ostream& out);
    void printLabels(ostream& out) const;
    bool readLabels(istream& in, vector< vector<VisionFieldObject*> >& labels) const;
    bool readLabels(istream& in, vector< vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& labels) const;

    map<VisionFieldObject::VFO_ID, pair<float, int> > evaluateFrame(const vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > >& ground_truth,
                                                                    const map<VisionFieldObject::VFO_ID, float>& false_pos_costs,
                                                                    const map<VisionFieldObject::VFO_ID, float>& false_neg_costs);

    map<VisionFieldObject::VFO_ID, Vector3<double> > precisionRecall(const vector<pair<VisionFieldObject::VFO_ID, Vector2<double> > >& ground_truth);


private:
    VisionControlWrapper();

    bool objectTypesMatch(VisionFieldObject::VFO_ID id0, VisionFieldObject::VFO_ID id1) const;

    static VisionControlWrapper* instance;  //! @var static singleton instance

    VisionController controller;           //! @var the system controller
    DataWrapper* data_wrapper;              //! @var the data wrapper
};

#endif // CONTROLWRAPPERTRAINING_H
