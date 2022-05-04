///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Motion Primitives
//  Workfile:        AssemblyPrims.cpp
//  Revision:        15 December, 2014
//  Author:          J. Marvel
//
//  Description
//  ===========
// 
///////////////////////////////////////////////////////////////////////////////

#include "AssemblyPrims.h"
#include <iostream>
//#include <stdio.h>
#define MAXBIT 30  //parameter defined for the Sobel sequence function
///////////////////////////////////////////////////////////////////////////////

double decimal (double val)
{
  int intval = (int)val;
  double returnme = (val - intval);
  if (returnme < 0.0f)
  {
    returnme += 1.0f;
  }
  return returnme;
}

int whole (double val)
{
  return (int)(val - decimal(val));
}

bool even(int val)
{
  return (decimal((double)val/2.0f) < 0.01f);
}

//! @brief Generates a Sobol sequence of random numbers;
//!
//! @param x    Array of pseudo-random numbers generated by this function
//! @param n    Number of pseudo-random numbers to generate
//! @param init TODO
//!
//! @note Modified from "Numerical Recipes in C" (https://www2.units.it/ipl/students_area/imm2/files/Numerical_Recipes.pdf)
//!
void sobseq(double *x, double *y, int init)
{
  int j, k, l;
  unsigned long i, im, ipp;
  static float fac;
  static unsigned long in, ix[3], *iu[MAXBIT + 1];
  static unsigned long mdeg[3] = {0,1,2};
  static unsigned long ip[3] = { 0,0,1};
  static unsigned long iv[2*MAXBIT + 1] = {0,1,1,1,1,1,1,3,1,3,3,1,1,5,7,7,3,3,5,15,11,5,15,13,9};
  
  if (init==0) 
  {  //Initialize the genereator the first time the function is called
    for (k = 1; k <= 2; k++)
    {
      ix[k] = 0;
    }
    in = 0;
    if (iv[1] != 1)
    {
      return;
    }
    fac = 1.0 / (1L << MAXBIT);
    for (j = 1, k = 0; j <= MAXBIT; j++, k += 2)
    {
      iu[j] = &iv[k];
    }
    //To allow both 1D and 2D addressing.
    for (k = 1; k <= 2; k++) 
    {
      for (j = 1; j <= mdeg[k]; j++)
      {
        iu[j][k] <<= (MAXBIT - j);
      }
      //Stored values only require normalization.
      for (j = mdeg[k] + 1; j <= MAXBIT; j++) 
      {
        //Use the recurrence to get other values.
        ipp = ip[k];
        i = iu[j - mdeg[k]][k];
        i ^= (i >> mdeg[k]);
        for (l = mdeg[k] - 1; l >= 1; l--)
        {
          if (ipp & 1) 
            i ^= iu[j - l][k];
          ipp >>= 1;
        }
          iu[j][k] = i;
      }
    }
  }
  //Calculate the pair of numbers in the 2D Sobel Sequence
  im = in++;
  for (j = 1; j <= MAXBIT; j++)
  { //Find the rightmost zero bit.
    if (!(im & 1)) break;
    im >>= 1;
  }
  if (j > MAXBIT)
  {
    printf("MAXBIT too small in sobseq");
  }
  im = (j - 1)*2;
  ix[1] ^= iv[im + 1];
  ix[2] ^= iv[im + 2];
  *x = (double)(ix[1] * fac);
  *y = (double)(ix[2] * fac);
}

///////////////////////////////////////////////////////////////////////////////

namespace MotionPrims
{
  LIBRARY_API Assembly::Assembly ()
  {
    curFreq_ = 10.0f;
    newSearch = true;
    sqs_x = 0;
    sqs_y = 0;
    //robot_ = new crpi_robot::CrpiRobot("universal_ur10_right.dat");
  }


  LIBRARY_API Assembly::~Assembly ()
  {
    termParams_.clear();
    assemblyParams_.clear();
    //robot_ = NULL;
  }
  

  LIBRARY_API CanonReturn Assembly::AddSearchRandom (double radius, bool walk)
  {
    assemblyParams aP;
    aP.sType = ASSEMBLY_RANDOM;
    aP.radius = radius;
    aP.randWalk = walk;
    return AddSearch (aP);
  }

  LIBRARY_API CanonReturn Assembly::AddSearchSobol(double radius, bool walk)
  {
    assemblyParams aP;
    aP.sType = ASSEMBLY_SOBOL;
    aP.radius = radius;
    aP.randWalk = walk;
    return AddSearch(aP);
  }


  LIBRARY_API CanonReturn Assembly::AddSearchSpiral (int turns, double radius, double speed)
  {
    assemblyParams aP;
    aP.sType = ASSEMBLY_SPIRAL;
    aP.speed = fabs(speed);
    aP.radius = fabs(radius);
    aP.turns = abs(turns);

    return AddSearch (aP);
  }
  
  LIBRARY_API CanonReturn Assembly::AddSearchSpiralOptimal(double clearance, double radius, double speed)
  {
    assemblyParams aP;
    aP.sType = ASSEMBLY_SPIRAL_OPTIMAL;
    aP.speed = fabs(speed);
    aP.radius = fabs(radius);
    aP.pitch = fabs(clearance); //ensures no point in search region is furthar from the spiral path than the clearance distance

    return AddSearch(aP);
  }

  LIBRARY_API CanonReturn Assembly::AddSearchSqSpiral(double lengthStep, double radius)
  {
    //std::cout << "Adding square spiral search" << endl;

    assemblyParams aP;
    aP.sType = ASSEMBLY_SQ_SPIRAL;
    aP.lengthStep = fabs(lengthStep);
    aP.radius = fabs(radius);

    // compute number of points
    int irad = fabs(radius / lengthStep);
    int sq_side = 2 * irad + 1;
    aP.totalPoints = sq_side * sq_side;

    // generate lookup table
    init_sqs_table(aP.totalPoints);

    return AddSearch(aP);
  }

  LIBRARY_API void Assembly::init_sqs_table(int table_size)
  {
    //! delete if arrays previously initialized
    if (sqs_x)
      delete[] sqs_x;
    if (sqs_y)
      delete[] sqs_y;

    sqs_x = new int[table_size];
    sqs_y = new int[table_size];

    //! (di, dj) is a vector - direction in which we move right now
    int di = 1;
    int dj = 0;
    //! length of current segment
    int segment_length = 1;

    //! current position (i, j) and how much of current segment we passed
    int i = 0;
    int j = 0;
    int segment_passed = 0;
    for (int k = 0; k < table_size; ++k)
    {
      //! make a step, add 'direction' vector (di, dj) to current position (i, j)
      i += di;
      j += dj;
      ++segment_passed;

      //! record offsets
      sqs_x[k] = i;
      sqs_y[k] = j;

      //System.out.println(i + " " + j);

      if (segment_passed == segment_length) {
        // done with current segment
        segment_passed = 0;

        // 'rotate' directions
        int buffer = di;
        di = -dj;
        dj = buffer;

        // increase segment length if necessary
        if (dj == 0) {
          ++segment_length;
        }
      }
    }
  }

  LIBRARY_API CanonReturn Assembly::AddSearchRaster (int rasters, double width, double length, double speed)
  {
    assemblyParams aP;
    aP.sType = ASSEMBLY_RASTER;
    aP.speed = fabs(speed);
    aP.width = fabs(width);
    aP.length = fabs(length);
    aP.turns = abs(rasters);

    return AddSearch (aP);
  }


  LIBRARY_API CanonReturn Assembly::AddSearchRotation (double range, double speed)
  {
    assemblyParams aP;
    aP.sType = ASSEMBLY_ROTATION;
    aP.speed = fabs(speed);
    aP.magnitude = fabs(range);

    return AddSearch (aP);
  }


  LIBRARY_API CanonReturn Assembly::AddSearchCircle (double radius, double speed)
  {
    assemblyParams aP;
    aP.sType = ASSEMBLY_CIRCLE;
    aP.speed = fabs(speed);
    aP.radius = fabs(radius);

    return AddSearch (aP);
  }


  LIBRARY_API CanonReturn Assembly::AddSearchHop (double magnitude, double frequency)
  {
    assemblyParams aP;

    aP.sType = ASSEMBLY_HOP;
    aP.speed = fabs(frequency);
    aP.magnitude = fabs(magnitude);

    return AddSearch (aP);
  }


  LIBRARY_API CanonReturn Assembly::AddSearchLinear (double xoff, double yoff, double zoff, double speed)
  {
    assemblyParams aP;

    aP.sType = ASSEMBLY_LINEAR;
    aP.speed = fabs(speed);
    aP.x = xoff;
    aP.y = yoff;
    aP.z = zoff;

    return AddSearch (aP);
  }


  LIBRARY_API CanonReturn Assembly::AddSearchConstOffset (double xoff, double yoff, double zoff)
  {
    assemblyParams aP;

    aP.sType = ASSEMBLY_CONST_OFFSET;
    aP.x = xoff;
    aP.y = yoff;
    aP.z = zoff;

    return AddSearch (aP);
  }
  

  LIBRARY_API CanonReturn Assembly::AddSearch (assemblyParams &aP)
  {
    if (motionCfg (aP))
    {
      assemblyParams_.push_back (aP);
      return CANON_SUCCESS;
    }

    return CANON_FAILURE;
  }

  LIBRARY_API bool Assembly::motionCfg (assemblyParams &aP)
  {
    /*
    //! Configure the offsets based on the calculated update frequency.
//    if (first_run)
//    {
//      cur_freq = get_freq(TimerNo, true);
//    }
//    else
//    {
//      cur_freq = get_freq(TimeNo, false);
//    }
    cur_freq = last_known_freq;
    */

    switch (aP.sType)
    {
    case ASSEMBLY_RANDOM:
    case ASSEMBLY_SOBOL:
      //! Ignore the update frequency, each motion needs to be in the range of the specified radius
      break;
    case ASSEMBLY_SPIRAL_OPTIMAL:
      aP.thetaMax = (aP.radius / aP.pitch) * 360.0f;
      aP.radiusOffset = aP.pitch / 360.0f;
      aP.degOffsetDelta = (aP.speed < 360.0f ? aP.speed : 360.0f) / curFreq_;
      break;
    case ASSEMBLY_SPIRAL:
      aP.thetaMax = aP.turns * 360.0f;
      aP.radiusOffset = aP.radius / aP.thetaMax;
      aP.degOffsetDelta = (aP.speed < 360.0f ? aP.speed : 360.0f) / curFreq_;
      break;
    case ASSEMBLY_SQ_SPIRAL:
      break;
    case ASSEMBLY_RASTER:
      if (even(aP.turns))
      {
        aP.lengthStep = aP.length / (double)(aP.turns - 1);
        aP.totalLength = ((double)aP.turns * aP.width) + aP.length;
      }
      else
      {
        aP.lengthStep = aP.length / (double)aP.turns;
        aP.totalLength = ((double)(aP.turns + 1.0f) * aP.width) + aP.length;
      }
      aP.lengthDelta = aP.speed / curFreq_;
      aP.rasterRatio = aP.width / (aP.width + aP.lengthStep);
      break;
    case ASSEMBLY_TILT:
      //! TODO
      break;
    case ASSEMBLY_ROTATION:
      aP.degOffsetDelta = (aP.speed < 360.0f ? aP.speed : 360.0f) / curFreq_;
      break;
    case ASSEMBLY_CIRCLE:
      aP.degOffsetDelta = (aP.speed < 360.0f ? aP.speed : 360.0f) / curFreq_;
      break;
    case ASSEMBLY_HOP:
      aP.degOffsetDelta = aP.speed * curFreq_;
      break;
    case ASSEMBLY_LINEAR:
      aP.totalLength = sqrt((aP.x * aP.x)+(aP.y * aP.y)+(aP.z * aP.z));
      aP.lengthStep = aP.speed / curFreq_;
      aP.lengthDelta = aP.totalLength / aP.speed;
      aP.xOffset = (aP.x / aP.lengthDelta) / curFreq_;
      aP.yOffset = (aP.y / aP.lengthDelta) / curFreq_;
      aP.zOffset = (aP.z / aP.lengthDelta) / curFreq_;
      break;
    case ASSEMBLY_CONST_OFFSET:
      //aP.speed = 30.0f;
      //aP.totalLength = sqrt((aP.x * aP.x)+(aP.y * aP.y)+(aP.z * aP.z));
      aP.lengthStep = aP.speed / curFreq_;
      aP.lengthDelta = aP.totalLength / aP.speed;
      aP.xOffset = aP.x;//(aP.x / aP.lengthDelta) / curFreq_;
      aP.yOffset = aP.y;//(aP.y / aP.lengthDelta) / curFreq_;
      aP.zOffset = aP.z;//(aP.z / aP.lengthDelta) / curFreq_;
      break;
    default:
      //! Unknown state
      return false;
      break;
    }
    return true;
  } //motionCfg


  LIBRARY_API CanonReturn Assembly::AddTerminator (terminatorParams &tP)
  {
    termParams_.push_back (tP);
    return CANON_SUCCESS;
  }


  LIBRARY_API CanonReturn Assembly::ClearSearch ()
  {
    termParams_.clear();
    assemblyParams_.clear();
    newSearch = true;
    return CANON_SUCCESS;
  } //ClearSearch


  LIBRARY_API CanonReturn Assembly::RunAssemblyStep (int counter, robotPose &robPose, robotPose &newPose, robotIO &ios)
  {
    bool state;
    CanonReturn returnMe = CANON_RUNNING;
    vector<terminatorParams>::iterator tpi;
    vector<assemblyParams>::iterator api;
    int index;

    //! known_freq = get_freq(10, new_search)

    state = true;
    curPose_ = robPose;
    curIO_ = ios;

    if (termParams_.size() < 1)
    {
      //! At least one termination condition must be defined
      return CANON_FAILURE;
    }

    if (newSearch)
    {
      initPose_ = robPose;

      for (tpi = termParams_.begin(); tpi != termParams_.end(); ++tpi)
      {
        if (tpi->tType == TERMINATOR_TIMER)
        {
          timer_.stopTimer();
          timer_.startTimer();
        }        
      } // for (tpi ...)
      newSearch = false;
    } // if (newSearch)

    newPose_ = initPose_;

    for (api = assemblyParams_.begin(); api != assemblyParams_.end(); ++api)
    {
      state = state && applyOffset(counter, *api);
    }

    if (state)
    {
      state &= testTerm (index);
    }
    else
    {
      newSearch = true;
      return CANON_FAILURE;
    }

    if (state)
    {
      newPose = newPose_;
      //! Termination condition not met, still searching
//      state &= (robot_->MoveTo(newPose_) == CANON_SUCCESS);
      //counter += 1;
    }
    else
    {
      //! Search terminated (success, error, fail)
      //! Type of terminator condition found at termParms_.at(index).tType
      returnMe = termParams_.at(index).rType;
      newSearch = true;
    }

    return returnMe;
  } //RunAssembly

  
  LIBRARY_API bool Assembly::applyOffset (int counter, assemblyParams &ap)
  {
    robotPose deltas;
    //! Initialize variables to 0
    deltas.x = deltas.y = deltas.z = deltas.xrot = deltas.yrot = deltas.zrot = 0.0f;
    int localCount;
    double radius;
    double degOffset;
    double position;
    double ratio;
    double temp;
    int wholeval;

    switch (ap.sType)
    {
    case ASSEMBLY_RANDOM:
      if (ap.randWalk)
      {
        deltas.x = (curPose_.x - initPose_.x);
        deltas.y = (curPose_.y - initPose_.y);
      }
    if (counter > 0) {
      deltas.x += (ap.radius * ((2500.0f - (double)(rand() % 5000)) / 2500.0f));
      deltas.y += (ap.radius * ((2500.0f - (double)(rand() % 5000)) / 2500.0f));
      //cout << endl << "Random Offset " << counter << ": (" << deltas.x << ", " << deltas.y << ")";
    }
      break;
  case ASSEMBLY_SOBOL:
    if (ap.randWalk)
    {
      deltas.x = (curPose_.x - initPose_.x);
      deltas.y = (curPose_.y - initPose_.y);
    }
    if (counter == 0) { sobseq(&deltas.x, &deltas.y, 0); }
    else {
      sobseq(&deltas.x, &deltas.y, 1);
      deltas.x = (ap.radius * ((0.5f - deltas.x) / 0.5f));
      deltas.y = (ap.radius * ((0.5f - deltas.y) / 0.5f));
      //cout << endl << "Sobol Offset " << counter << ": (" << deltas.x << ", " << deltas.y << ")";
    }
    break;
  case ASSEMBLY_SPIRAL:
  case ASSEMBLY_SPIRAL_OPTIMAL:
    localCount = counter % (int)(ap.thetaMax / ap.degOffsetDelta);
      degOffset = localCount * ap.degOffsetDelta;
      radius = degOffset * ap.radiusOffset;
    //cout << endl << "Helix Iteration " << counter << "   Degrees: " << degOffset << "   Radius: " << radius;
      deltas.x = radius * cos(degOffset*(3.141592654/180.0f));
      deltas.y = radius * sin(degOffset*(3.141592654/180.0f));
      break;
  case ASSEMBLY_SQ_SPIRAL:
    if (counter < ap.totalPoints)
    {
      deltas.x = sqs_x[counter] * ap.lengthStep;
      deltas.y = sqs_y[counter] * ap.lengthStep;

      //std::cout << "Square Spiral: total = " << ap.totalPoints << ", count = " << counter << ", x = " << deltas.x << ", y = " << deltas.y << std::endl;
    }
    break;
  case ASSEMBLY_RASTER:
      temp = ap.totalLength / ap.lengthDelta;
      localCount = counter % (int)(2.0f * temp);

      if (localCount > temp)
      {
        localCount = (int)(2.0f * temp) - localCount;
      }

      position = localCount * ap.lengthDelta;
      ratio = position / (ap.width + ap.lengthStep);
      
      temp = decimal(ratio);
      wholeval = whole(ratio);

      if (temp > ap.rasterRatio)
      {
        //! In the raster step
        if (even(wholeval))
        {
          deltas.y = ap.width;
        }
        else
        {
          deltas.y = 0.0f;
        }
        deltas.x = (wholeval + ((temp - ap.rasterRatio) / (1.0 - ap.rasterRatio))) * ap.lengthStep;
      }
      else
      {
        //! On the raster
        if (even(wholeval))
        {
          deltas.y = (temp / ap.rasterRatio) * ap.width;
        }
        else
        {
          deltas.y = (1.0f - (temp / ap.rasterRatio)) * ap.width;
        }
        deltas.x = wholeval * ap.lengthStep;
      }
      break;
    case ASSEMBLY_TILT:
      //! TODO
      break;
    case ASSEMBLY_ROTATION:
      deltas.zrot = ap.magnitude * sin((counter * ap.degOffsetDelta)*(3.141592654/180.0f));
      break;
    case ASSEMBLY_CIRCLE:
      localCount = counter % (int)(360.0f / ap.degOffsetDelta);
      temp = localCount * ap.degOffsetDelta;
      deltas.x = ap.radius - (ap.radius * cos(temp));
      deltas.y = ap.radius * sin(temp*(3.141592654/180.0f));
      break;
    case ASSEMBLY_HOP:
      deltas.z = ap.magnitude * sin((counter * ap.degOffsetDelta)*(3.141592654/180.0f));
      break;
    case ASSEMBLY_LINEAR:
      temp = ap.totalLength / ap.lengthStep;
      localCount = counter % (int)(2.0f * temp);
      if (localCount > temp)
      {
        localCount = (int)(2.0f * temp) - localCount;
      }
      deltas.x = localCount * ap.xOffset;
      deltas.y = localCount * ap.yOffset;
      deltas.z = localCount * ap.zOffset;
      break;
    case ASSEMBLY_CONST_OFFSET:
      //temp = ap.totalLength / ap.lengthStep;
      if (counter < 2)
      {
        deltas.x = counter * ap.xOffset;
        deltas.y = counter * ap.yOffset;
        deltas.z = counter * ap.zOffset;
      }
      else
      {
        deltas.x += ap.xOffset;
        deltas.y += ap.yOffset;
        deltas.z += ap.zOffset;
      }
      break;
    default:
      //! ERROR
      return false;
      break;
    }

    newPose_ = newPose_ + deltas;
    //newPose_.x += deltas.x;
    //newPose_.y += deltas.y;
    return true;
  } // applyOffset


  LIBRARY_API CanonReturn Assembly::AddTerminatorTimer (CanonReturn rType, double timeout)
  {
    terminatorParams tP;
    tP.tType = TERMINATOR_TIMER;
    tP.rType = rType;
    tP.endTime = fabs(timeout);
    
    return AddTerminator(tP);
  }

  LIBRARY_API CanonReturn Assembly::AddTerminatorContact (CanonReturn rType, double threshold)
  {
    terminatorParams tP;

    tP.tType = TERMINATOR_CONTACT;
    tP.rType = rType;
    tP.threshold = fabs(threshold);
    
    return AddTerminator(tP);
  }

  LIBRARY_API CanonReturn Assembly::AddTerminatorSignal (CanonReturn rType, int signal)
  {
    terminatorParams tP;
    tP.tType = TERMINATOR_EXTSIGNAL;
    tP.rType = rType;
    tP.signal = signal;

    return AddTerminator(tP);
  }


  LIBRARY_API CanonReturn Assembly::AddTerminatorDistance (CanonReturn rType, double x, double y, double z, double total)
  {
    terminatorParams tP;
    tP.tType = TERMINATOR_DISTANCE;
    tP.rType = rType;
    if (x <= 0.0f)
    {
      tP.xDelta = -1.0f;
    }
    else
    {
      tP.xDelta = fabs(x);
    }

    if (y <= 0.0f)
    {
      tP.yDelta = -1.0f;
    }
    else
    {
      tP.yDelta = fabs(y);
    }

    if (z <= 0.0f)
    {
      tP.zDelta = -1.0f;
    }
    else
    {
      tP.zDelta = fabs(z);
    }

    if (total <= 0.0f)
    {
      tP.threshold = -1.0f;
    }
    else
    {
      tP.threshold = fabs(total);
    }

    return AddTerminator(tP);
  }


  LIBRARY_API CanonReturn Assembly::AddTerminatorRepetition (CanonReturn rType, int reps)
  {
    terminatorParams tP;
    
    tP.tType = TERMINATOR_REPETITION;
    tP.rType = rType;
    tP.timer = reps;
    return AddTerminator(tP);
  }


  LIBRARY_API bool Assembly::testTerm (int &index)
  {
    index = -1;
    unsigned int x;
    double dx, dy, dz, dp;

    for (x = 0; x < termParams_.size(); ++x)
    {
      switch (termParams_.at(x).tType)
      {
      case TERMINATOR_TIMER:
        termParams_.at(x).result = (timer_.timeElapsed() >= termParams_.at(x).endTime);
        if (termParams_.at(x).result)
        {
          timer_.stopTimer();
          index = x;
          return false;
        }
        break;
      case TERMINATOR_EXTSIGNAL:
        termParams_.at(x).result = curIO_.dio[termParams_.at(x).signal];
        if (termParams_.at(x).result)
        {
          index = x;
          return false;
        }
        break;
      case TERMINATOR_CONTACT:
        /*
        TODO
        TERMINATORS[COUNTER].RESULT_ = TERMINATORS[COUNTER].RESULT_ OR (ABS($TORQUE_TCP_EST.FT.Z) >= TERMINATORS[COUNTER].THRESHOLD_)
        */
        break;
      case TERMINATOR_DISTANCE:
        dx = fabs(initPose_.x - curPose_.x);
        dy = fabs(initPose_.y - curPose_.y);
        dz = fabs(initPose_.z - curPose_.z);
        dp = sqrt((dx * dx) + (dy * dy) + (dz * dz));

        if (termParams_.at(x).zDelta > 0.0f)
        {
          termParams_.at(x).result = (termParams_.at(x).zDelta <= dz);
        }
        else if (termParams_.at(x).threshold > 0.0f)
        {
          termParams_.at(x).result = (termParams_.at(x).threshold <= dp);
        }

        if (termParams_.at(x).result)
        {
          index = x;
          return false;
        }
        break;
      case TERMINATOR_REPETITION:
        //! TODO
        break;
      case TERMINATOR_NONE:
      default:
        termParams_.at(x).result = false;
        //! Error: Termination condition has not been defined
        break;
      } //switch (termParams_.at(index).tType)
    } // for (x = 0; x < termParams_.size(); ++x)
    return true;
  } // testTerm

} // namespace MotionPrims