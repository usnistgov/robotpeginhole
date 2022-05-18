#include "test_gazebo_package/wm.h"

double gzPegboardScaleFactor = 0.002;
double dGzLengthPeg = 8.8 * gzPegboardScaleFactor; // in meters - scale is 0.002
// Fanuc taskboard peg array of holes
// 6.35mm=1/2 in   50.8=2in
double zWrldMinPegArray = 0.991;     // GZ world coord in meters as reported by gazebo
double zSizePegArray = 6.35 * 0.002; // in meters (scale is 0.002 mm->meters) twice as big
double zWrldMaxPegArray = 1.0799;    // GZ zMinPegArray+zSizePegArray;  // in meters using calculator
double dDwellTime = 0.0;
double xfudge = 0.02;

/**
 * @brief  static definition for CPegArrayHole. Used when
 * CPegArrayHole is empty or not found.
 */
static CPegArrayHole empty1;

/**
 * @brief empty reference for CPegArrayHole. Used when
 * CPegArrayHole is empty or not found.
 * 
 * @return CPegArrayHole& 
 */
CPegArrayHole &emptyhole(empty1);

/**
 * @brief  peg supply definition of the Gazebo world locations of pegarray with pegs in holes
 * 
 */
std::vector<CPegArrayHole> p1 = {
    CPegArrayHole("Peg1", "Full", "round", tf::Vector3(0.2697, -1.1046, 0.9307)),
    CPegArrayHole("Peg2", "Full", "square", tf::Vector3(0.2697, -1.0538, 0.9307)),
    CPegArrayHole("Peg3", "Full", "round", tf::Vector3(0.2697, -1.003, 0.9307)),
    CPegArrayHole("Peg4", "Full", "square", tf::Vector3(0.2189, -1.1046, 0.941723)),
    CPegArrayHole("Peg5", "Full", "round", tf::Vector3(0.2189, -1.0538, 0.941723)),
    CPegArrayHole("Peg6", "Full", "square", tf::Vector3(0.21897, -1.003, 0.941723)),
    CPegArrayHole("Peg7", "Full", "round", tf::Vector3(0.1681, -1.1046, 0.9544)),
    CPegArrayHole("Peg8", "Full", "square", tf::Vector3(0.1681, -1.0538, 0.9544)),
    CPegArrayHole("Peg9", "Full", "round", tf::Vector3(0.1681, -1.003, 0.9544))};

/**
     * @brief  collection of all pegs in supply pegarray
     * 
     * @return CPegHoleArray 
     */
CPegHoleArray pegs("pegsarray", p1);

/**
 * @brief define the Gazebo world locations of empty pegboard with empty holes
 */
std::vector<CPegArrayHole> p2 = {
    CPegArrayHole("Hole1", "Open", "round", tf::Vector3(0.2697, -1.3546, 0.9307)),
    CPegArrayHole("Hole2", "Open", "square", tf::Vector3(0.2697, -1.3038, 0.9307)),
    CPegArrayHole("Hole3", "Open", "round", tf::Vector3(0.2697, -1.253, 0.9307)),
    CPegArrayHole("Hole4", "Open", "square", tf::Vector3(0.2189, -1.3546, 0.941723)),
    CPegArrayHole("Hole5", "Open", "round", tf::Vector3(0.2189, -1.3038, 0.941723)),
    CPegArrayHole("Hole6", "Open", "square", tf::Vector3(0.21897, -1.253, 0.941723)),
    CPegArrayHole("Hole7", "Open", "round", tf::Vector3(0.1681, -1.3546, 0.9544)),
    CPegArrayHole("Hole8", "Open", "square", tf::Vector3(0.1681, -1.3038, 0.9544)),
    CPegArrayHole("Hole9", "Open", "round", tf::Vector3(0.1681, -1.253, 0.9544))};

/**
     * @brief  collection of all world location holes in destination pegarray
     * 
     * @return CPegHoleArray 
     */
CPegHoleArray holes("holesarray",p2);

/**
 * @brief definiiton of local offset locations of holes in peg array
 */
std::vector<CPegArrayHole> p3 = {
    CPegArrayHole("OffsetPeg1", "Full", "round", tf::Vector3(-0.0508, -0.0508, 0.0143)),
    CPegArrayHole("OffsetPeg2", "Open", "square", tf::Vector3(-0.0508, 0., 0.)),
    CPegArrayHole("OffsetPeg3", "Full", "round", tf::Vector3(-0.0508, 0.0508, 0.0143)),
    CPegArrayHole("OffsetPeg4", "Open", "square", tf::Vector3(0., -0.0508, 0.0143)),
    CPegArrayHole("OffsetPeg5", "Full", "round", tf::Vector3(0., 0., 0.)),
    CPegArrayHole("OffsetPeg6", "Open", "square", tf::Vector3(0., 0.0508, -0.0129)),
    CPegArrayHole("OffsetPeg7", "Full", "round", tf::Vector3(0.0508, -0.0508, 0.0143)),
    CPegArrayHole("OffsetPeg8", "Open", "square", tf::Vector3(0.0508, 0., 0.)),
    CPegArrayHole("OffsetPeg9", "Full", "round", tf::Vector3(0.0508, 0.0508, -0.0129))};

    std::vector<CPegArrayHole> p4 = {
     CPegArrayHole("OffsetHole1", "Full", "round", tf::Vector3(-0.0508, -0.0508, 0.0143)),
    CPegArrayHole("OffsetHole2", "Open", "square", tf::Vector3(-0.0508, 0., 0.)),
    CPegArrayHole("OffsetHole3", "Full", "round", tf::Vector3(-0.0508, 0.0508, 0.0143)),
    CPegArrayHole("OffsetHole4", "Open", "square", tf::Vector3(0., -0.0508, 0.0143)),
    CPegArrayHole("OffsetHole5", "Full", "round", tf::Vector3(0., 0., 0.)),
    CPegArrayHole("OffsetHole6", "Open", "square", tf::Vector3(0., 0.0508, -0.0129)),
    CPegArrayHole("OffsetHole7", "Full", "round", tf::Vector3(0.0508, -0.0508, 0.0143)),
    CPegArrayHole("OffsetHole8", "Open", "square", tf::Vector3(0.0508, 0., 0.)),
    CPegArrayHole("OffsetHole9", "Full", "round", tf::Vector3(0.0508, 0.0508, -0.0129))};

CPegArrayHoleOffsets supplybasearray("supplybasearray",p3);
CPegArrayHoleOffsets supplypegarray("supplypegarray",p3);
CPegArrayHoleOffsets destbasearray("destbasearray",p4);
CPegArrayHoleOffsets destpegarray("destpegarray",p4);

#ifdef PEGARRAY
// DO NOT PUT CPegArray due to static init order. BIG PROBLEM. IGNORED FOR NOW.
// CPegArray holearray("insertion.pegholesarray");
// CPegArray pegarray("supply.holepegarray");
#endif

#ifdef PEGARRAY
// std::map<std::string, CPegArray *> pegarray_names;
// CPegArray::CPegArray(std::string name)
// {
//     this->name = name;
//     pegarray_names[name] = this;
//     // // This is established pegarray dimensions, 4/1/2022
//     scale = 0.002;
//     xlen = 44.45000076293945 * 2. * scale;
//     ylen = 3.174999237060547 * 2. * scale;
//     zlen = 88.90000915527344 * scale;
// }
// CPegArray &CPegArray::find(std::string name)
// {
//     static CPegArray dummy("dummy.pegarray");
//     if (pegarray_names.find(name) != pegarray_names.end())
//         return *pegarray_names[name];
//     return dummy;
// }
#endif
