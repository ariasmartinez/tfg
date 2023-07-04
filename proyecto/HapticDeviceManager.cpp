/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name: 

  HapticDeviceManager.cpp

Description:

  Implements the IHapticDeviceManager interface.

*******************************************************************************/


#include ".\include\PointManipulationAfx.h"

#include ".\include\HapticDeviceManager.h"
#include ".\include\PointManager.h"
#include ".\include\PointManipulationUtil.h"

#include ".\include\ViewApparentPointConstraint.h"
#include ".\include\PlaneWithAxesConstraint.h"

#include <cassert>
#include <algorithm>

#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HDU/hduMath.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduHapticDevice.h>


#include <SnapConstraints/ISnapConstraintsAPI.h>
#include <SnapConstraints/SnapConstraint.h>
#include <SnapConstraints/PlaneConstraint.h>
#include <SnapConstraints/StickToConstraint.h>

#include <iostream>
#include <fstream>

#include <chrono>

#define CURSOR_SIZE_PIXELS  15
# define M_PI           3.14159265358979323846  /* pi */

#define BEGIN_DEVICE_UPDATE_PRIORITY    (HD_MAX_SCHEDULER_PRIORITY)
#define RENDER_EFFECT_FORCE_PRIORITY    (HD_MAX_SCHEDULER_PRIORITY - 1)
#define END_DEVICE_UPDATE_PRIORITY      (HD_MAX_SCHEDULER_PRIORITY - 2)
#define SYNCHRONIZE_STATE_PRIORITY      HD_DEFAULT_SCHEDULER_PRIORITY

using namespace SnapConstraints;

using namespace std;

namespace
{

/*******************************************************************************
 HapticDeviceManager
 
 This class.provides the application glue that marries haptic interactions with
 the application logic/state. The HapticDeviceManager knows how to interact
 with the points maintained by the IPointManager instanc
 e to support
 interactions like snapping, highlighting, and manipulation of points. This class
 also orchestrates the synchronization between haptic and graphics state.
 Additionally, this class handles the workspace mapping that allows us to
 perform transformations between device coordinates and world coordinates.
*******************************************************************************/
class HapticDeviceManager : public IHapticDeviceManager
{
public:

    HapticDeviceManager();
    ~HapticDeviceManager();

    void setup(IPointManager *pPointManager);
    void cleanup();

    void updateState();
    void updateWorkspace();

    void drawCursor();

    void setManipulationStyle(unsigned style);
    unsigned getManipulationStyle() const { return m_manipStyle; }

private:

    void updateSnapping();

    void startManipulation(int nManipPoint);
    void stopManipulation();
    void updateManipulation();
    bool isManipulating() const { return m_nManipPoint >= 0; }
    void setMVP();

    static HDCallbackCode HDCALLBACK beginUpdateCallback(void *pUserData);
    static HDCallbackCode HDCALLBACK endUpdateCallback(void *pUserData);
    static HDCallbackCode HDCALLBACK springManipulationCallback(void *pUserData);
    static HDCallbackCode HDCALLBACK frictionManipulationCallback(void *pUserData);    
    static HDCallbackCode HDCALLBACK setDeviceTransformCallback(void *pUserData);
    static HDCallbackCode HDCALLBACK setConstraintCallback(void *pUserData);
    static HDCallbackCode HDCALLBACK clearConstraintCallback(void *pUserData);
    static HDCallbackCode HDCALLBACK CoulombCallback(void* pUserData);
    static HDCallbackCode HDCALLBACK clearForceCallback(void* pUserData);
    static HDCallbackCode HDCALLBACK actionInitialized(void* pUserData);
    static HDCallbackCode HDCALLBACK actionFinished(void* pUserData);
    
    static HDCallbackCode HDCALLBACK FrictionlessPlaneCallback(void* pUserData);

    static void madeContactCallbackGT(IHapticDevice::EventType event,
                     const IHapticDevice::IHapticDeviceState * const pState,
                     void *pUserData);

    static void lostContactCallbackGT(IHapticDevice::EventType event,
                     const IHapticDevice::IHapticDeviceState * const pState,
                     void *pUserData);

    static void buttonDownCallbackGT(IHapticDevice::EventType event,
                     const IHapticDevice::IHapticDeviceState * const pState,
                     void *pUserData);

    static void buttonUpCallbackGT(IHapticDevice::EventType event,
                     const IHapticDevice::IHapticDeviceState * const pState,
                     void *pUserData);

    static void buttonDownCallbackHT(IHapticDevice::EventType event,
                     const IHapticDevice::IHapticDeviceState * const pState,
                     void *pUserData);

    static void buttonUpCallbackHT(IHapticDevice::EventType event,
                     const IHapticDevice::IHapticDeviceState * const pState,
                     void *pUserData);

    static void errorCallbackGT(IHapticDevice::EventType event,
                     const IHapticDevice::IHapticDeviceState * const pState,
                     void *pUserData);

    HHD m_hHD;
    HDSchedulerHandle m_hBeginUpdateCallback;
    HDSchedulerHandle m_hEndUpdateCallback;
    HDSchedulerHandle m_hManipulationCallback;

    HDSchedulerHandle m_hPlaneCallback;

    IHapticDevice *m_pHapticDeviceHT;
    IHapticDevice *m_pHapticDeviceGT;

    IPointManager *m_pPointManager;

    int m_nManipPoint;
    ManipulationStyle m_manipStyle;

    hduVector3Dd m_deviceAnchor;

    ISnapConstraintsAPI *m_pSnapAPI;
    ViewApparentPointConstraint *m_pConstraint;

    hduVector3Dd m_effectForce;

    hduMatrix m_workspaceXform;
    hduMatrix m_offsetXform;

    HDdouble m_cursorScale;
    hduVector3Dd m_cameraPosWC;

    GLuint m_nCursorDisplayList;

    chrono::steady_clock::time_point begin_time;

    bool action_initialized = false;
    int actual_action = 0;

    hduVector3Dd actual_action_position = hduVector3Dd(0, 0, 0);

    int total_actions = 0;

    enum Mode {
        STARTING,
        NOFORCE,
        FORCE,
    };

    GLfloat red = 1.0;
    GLfloat green = 1.0;
    GLfloat blue = 1.0;

    bool valid = true;

    clock_t start_inside_point;


    int counter_accions = 0;

    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    hduVector3Dd last_position = hduVector3Dd(0,0,0);
    int filter = 0;
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
    std::chrono::steady_clock::time_point total_is_in_contact = TimePoint::max();

    int max_repetitions = 80;
    
    hduMatrix matrix_MVP;
    GLdouble projection_matrix[16];
    GLdouble view_matrix[16];
    float viewportHeight;
    float viewportWidth;
    float viewSize[2];
    float viewOffset[2];
    GLint viewport_matrix[4];


    int next_point = 0;
    int point_repetition = 0;
};

}

/*******************************************************************************
 Factory method for creating an instance of the HapticDeviceManager
*******************************************************************************/
IHapticDeviceManager *IHapticDeviceManager::create()
{
    return new HapticDeviceManager;
}

/*******************************************************************************
 Factory method for destroying an instance of the HapticDeviceManager
*******************************************************************************/
void IHapticDeviceManager::destroy(IHapticDeviceManager *&pInterface)
{
    if (pInterface)
    {
        HapticDeviceManager *pImp = static_cast<HapticDeviceManager *>(pInterface);
        delete pImp;
        pInterface = 0;
    }
}

/*******************************************************************************
 HapticDeviceManager Constructor
*******************************************************************************/
HapticDeviceManager::HapticDeviceManager() :
    m_hHD(HD_INVALID_HANDLE),
    m_hBeginUpdateCallback(HD_INVALID_HANDLE),
    m_hEndUpdateCallback(HD_INVALID_HANDLE),
    m_hManipulationCallback(HD_INVALID_HANDLE),
    m_hPlaneCallback(HD_INVALID_HANDLE),
    m_pHapticDeviceHT(0),
    m_pHapticDeviceGT(0),
    m_pPointManager(0),
    m_nManipPoint(-1),
    m_pSnapAPI(0),
    m_pConstraint(0),
    m_nCursorDisplayList(0)
{
}

/*******************************************************************************
 HapticDeviceManager Destructor
*******************************************************************************/
HapticDeviceManager::~HapticDeviceManager()
{
}

/*******************************************************************************
 This is the main initialization needed for the haptic glue code.
*******************************************************************************/
void HapticDeviceManager::setup(IPointManager *pPointManager)
{
 
    HDErrorInfo error;

    m_hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        std::cerr << error << std::endl;
        std::cerr << "Failed to initialize haptic device" << std::endl;
        std::cerr << "Press any key to quit." << std::endl;
        getchar();
        exit(-1);
    }

    /* Create the IHapticDevice instances for the haptic and graphic threads
       These interfaces are useful for handling the synchronization of
       state between the two main threads. */
    m_pHapticDeviceHT = IHapticDevice::create(
        IHapticDevice::HAPTIC_THREAD_INTERFACE, m_hHD);
    m_pHapticDeviceGT = IHapticDevice::create(
        IHapticDevice::GRAPHIC_THREAD_INTERFACE, m_hHD);

    /* Setup callbacks so we can be notified about events in the graphics 
       thread. */
    m_pHapticDeviceGT->setCallback(
        IHapticDevice::MADE_CONTACT, madeContactCallbackGT, this);
    m_pHapticDeviceGT->setCallback(
        IHapticDevice::LOST_CONTACT, lostContactCallbackGT, this);
    m_pHapticDeviceGT->setCallback(
        IHapticDevice::BUTTON_1_DOWN, buttonDownCallbackGT, this);    
   
    /* Setup callbacks so we can be notified about events in the haptics 
       thread. */
    m_pHapticDeviceHT->setCallback(
        IHapticDevice::BUTTON_1_DOWN, buttonDownCallbackHT, this);    
    m_pHapticDeviceHT->setCallback(
        IHapticDevice::BUTTON_1_UP, buttonUpCallbackHT, this);    

    m_pSnapAPI = ISnapConstraintsAPI::create();
    m_pConstraint = new ViewApparentPointConstraint(false);

    m_pPointManager = pPointManager;



    hdEnable(HD_FORCE_OUTPUT);

    m_hBeginUpdateCallback = hdScheduleAsynchronous(
        beginUpdateCallback, this, BEGIN_DEVICE_UPDATE_PRIORITY);
    m_hEndUpdateCallback = hdScheduleAsynchronous(
        endUpdateCallback, this, END_DEVICE_UPDATE_PRIORITY);

    /* Start the scheduler to get the servo loop going. */
    hdStartScheduler();
    
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        std::cerr << error << std::endl;
        std::cerr << "Failed to start scheduler" << std::endl;
        std::cerr << "Press any key to quit." << std::endl;
        getchar();
        exit(-1);
    }
}

/*******************************************************************************
 Reverse the setup process by shutting down and destructing the services
 used by the HapticDeviceManager.
*******************************************************************************/
void HapticDeviceManager::cleanup()
{
    hdStopScheduler();

    results.close();

    if (m_hBeginUpdateCallback != HD_INVALID_HANDLE)
    {
        hdUnschedule(m_hBeginUpdateCallback);
        m_hBeginUpdateCallback = HD_INVALID_HANDLE;
    }

    if (m_hPlaneCallback != HD_INVALID_HANDLE)
    {
        hdUnschedule(m_hPlaneCallback);
        m_hPlaneCallback = HD_INVALID_HANDLE;
    }

    if (m_hEndUpdateCallback != HD_INVALID_HANDLE)
    {
        hdUnschedule(m_hEndUpdateCallback);
        m_hEndUpdateCallback = HD_INVALID_HANDLE;
    }

    if (m_hHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(m_hHD);
        m_hHD = HD_INVALID_HANDLE;
    }

    IHapticDevice::destroy(m_pHapticDeviceGT);
    IHapticDevice::destroy(m_pHapticDeviceHT);

    ISnapConstraintsAPI::destroy(m_pSnapAPI);
    delete m_pConstraint;

    glDeleteLists(m_nCursorDisplayList, 1);
}

/*******************************************************************************
 This method will get called every tick of the graphics loop. It is primarily
 responsible for synchronizing state with the haptics thread as well as
 updating snap and manipulation state.
*******************************************************************************/
void HapticDeviceManager::updateState()
{
   
    /* Capture the latest state from the servoloop. */
    m_pHapticDeviceGT->beginUpdate(m_pHapticDeviceHT);
    m_pHapticDeviceGT->endUpdate(m_pHapticDeviceHT);

    updateSnapping();   
    updateManipulation();
    setMVP();
}

/*******************************************************************************
 This method works with the PointManager to determine if there are any points
 within proximity to the haptic device for snapping. This can be evaluated at
 the graphics rate since snapping is determined as a proximity condition. Plus,
 humans only make controlled motions at ~10-15 Hz, so it would be wasteful to
 check for the snapping condition any faster.
*******************************************************************************/
void HapticDeviceManager::updateSnapping()
{
    IHapticDevice::IHapticDeviceState *pState = 
        m_pHapticDeviceGT->getCurrentState();

    /* Don't disturb snap state if we're already in contact. */
    if (pState->isInContact())
        return;

    hduMatrix parentTworld(pState->getParentCumulativeTransform());
    hduMatrix worldTparent = parentTworld.getInverse();

    /* Get the local position of the device. */
    hduVector3Dd devicePositionLC = pState->getPosition();

    /* Setup the point and line constraint for each. */
    PointConstraint *pPointConstraint = m_pConstraint->getPointConstraint();
    LineConstraint *pLineConstraint = m_pConstraint->getLineConstraint();

    /* Create a view apparent constraint by using a line that passes from
       the eye position through the point of interest. This allows
       the user to select the point just by being overtop of it in the view. */
    hduVector3Dd cameraPosLC;
    worldTparent.multVecMatrix(m_cameraPosWC, cameraPosLC);
    pLineConstraint->setStartPoint(cameraPosLC);
    double minDist = m_pConstraint->getSnapDistance();
    int nClosestPoint = -1;    

    /* Search for the constraint that is closest to the device position. */
    int nPoints = m_pPointManager->getNumPoints();
    for (int i = 0; i < nPoints; i++)
    {
        /* Don't attempt to snap to the manipulated point. */
        if (i == m_nManipPoint)
            continue;

        hduVector3Dd proxyPositionLC;        
        double dist;

        /* Compute position of the test point in device coordinates. */
        hduVector3Dd pointPositionWC = m_pPointManager->getPointPosition(i);
        hduVector3Dd pointPositionLC;
        worldTparent.multVecMatrix(pointPositionWC, pointPositionLC);

        pPointConstraint->setPoint(pointPositionLC);
        pLineConstraint->setEndPoint(pointPositionLC);
        
        dist = m_pConstraint->testConstraint(devicePositionLC, proxyPositionLC);
        if (dist < minDist)
        {
            minDist = dist;
            nClosestPoint = i;
        }
    }

    /* Finally set the constraint based on the closest point. */
    if (nClosestPoint >= 0)
    {
        /* Compute position of the test point in device coordinates. */
        hduVector3Dd pointPositionWC = m_pPointManager->
            getPointPosition(nClosestPoint);
        hduVector3Dd pointPositionLC;
        worldTparent.multVecMatrix(pointPositionWC, pointPositionLC);

        pPointConstraint->setPoint(pointPositionLC);
        pLineConstraint->setEndPoint(pointPositionLC);

        /* Set the point index as the constraint private data, so the
           event handlers can know which point is being referenced. */
        m_pConstraint->setUserData((void *) nClosestPoint);

        hdScheduleSynchronous(setConstraintCallback, this,
            SYNCHRONIZE_STATE_PRIORITY);
    }
}

/*******************************************************************************
 Uses the current OpenGL viewing transforms to determine a mapping from device
 coordinates to world coordinates.
*******************************************************************************/
void HapticDeviceManager::updateWorkspace()
{
    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    /* Compute the transform for going from device coordinates to world
       coordinates based on the current viewing transforms. */
    hduMapWorkspaceModel(modelview, projection, m_workspaceXform);

    /* Compute the scale factor that can be applied to a unit sized object
       in world coordinates that will make it a particular size in pixels. */
    HDdouble screenTworkspace = hduScreenToWorkspaceScale(
        modelview, projection, viewport, m_workspaceXform);
    
    m_cursorScale = CURSOR_SIZE_PIXELS * screenTworkspace;

    /* Compute the updated camera position in world coordinates. */
    hduMatrix worldTeye(modelview);
    hduMatrix temp_model(modelview);
    hduMatrix temp_proj(projection);
    hduMatrix eyeTworld = worldTeye.getInverse();
    eyeTworld.multVecMatrix(hduVector3Dd(0, 0, 0), m_cameraPosWC);

    hdScheduleSynchronous(setDeviceTransformCallback, this,
                          SYNCHRONIZE_STATE_PRIORITY);    
}

/*******************************************************************************
 Draws a 3D cursor using the current device transform and the workspace
 to world transform.
*******************************************************************************/
void HapticDeviceManager::drawCursor()
{

    IHapticDevice::IHapticDeviceState *pState = 
        m_pHapticDeviceGT->getCurrentState();

 


    if (action_initialized && (getManipulationStyle() == NOFORCE_MANIPULATION_NOCURSOR || getManipulationStyle() == FORCE_MANIPULATION_NOCURSOR))
        return;

    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
    glPushMatrix();

    if (!m_nCursorDisplayList)
    {
        m_nCursorDisplayList = glGenLists(1);
        glNewList(m_nCursorDisplayList, GL_COMPILE);
        GLUquadricObj *qobj = gluNewQuadric();
        gluSphere(qobj, 0.5, 10, 10);
        gluDeleteQuadric(qobj);
        glEndList();
    }

    glMultMatrixd(pState->getParentCumulativeTransform());

    glMultMatrixd(pState->getTransform());

    glScaled(m_cursorScale, m_cursorScale, m_cursorScale);

    glEnable(GL_COLOR_MATERIAL);
    glColor3f(red, green, blue);

    glCallList(m_nCursorDisplayList);

    glPopMatrix(); 
    glPopAttrib();
}

/*******************************************************************************
 Sets the style of manipulation to be used when the user initiates a drag
 of a point. The style dictates what kind of force feedback will be provided.
*******************************************************************************/
void HapticDeviceManager::setManipulationStyle(unsigned style)
{

    if (style < INVALID_MANIPULATION)
    {
        m_manipStyle = (ManipulationStyle) style;
    }
}

/*******************************************************************************
 Start manipulation by assigning an index for a point to manipulate.
*******************************************************************************/
void HapticDeviceManager::startManipulation(int nManipPoint)
{
    assert(!isManipulating());

    m_nManipPoint = nManipPoint;
}

/*******************************************************************************
 Stop manipulation by setting the manipulated point index to an invalid value.
*******************************************************************************/
void HapticDeviceManager::stopManipulation()
{
    m_nManipPoint = -1;
}

/*******************************************************************************
 Update the position of the manipulated point. The position will either be
 set as the current position of the device, or the position of the contacted
 snap point.
*******************************************************************************/
void HapticDeviceManager::updateManipulation()
{
    if (isManipulating())
    {
        IHapticDevice::IHapticDeviceState *pState = 
            m_pHapticDeviceGT->getCurrentState();
    
        /* Get the current proxy location of the device in world coordinates
           and update the position of the manipulated point. */
        hduVector3Dd pointPositionWC;
        hduMatrix parentTworld(pState->getParentCumulativeTransform());

        if (pState->isInContact() && 
            getManipulationStyle() != FREE_MANIPULATION)
        {
            /* Colocate the manipulated point with the snapped point. */
            long int index = (long int) pState->getContactData();    

            pointPositionWC = m_pPointManager->getPointPosition(index);
        }
        else
        {
            parentTworld.multVecMatrix(pState->getProxyPosition(), pointPositionWC);
        }

        m_pPointManager->setPointPosition(m_nManipPoint, pointPositionWC);
    }
}

void HapticDeviceManager::setMVP()
{
    glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
    glGetDoublev(GL_MODELVIEW_MATRIX, view_matrix);
    glGetIntegerv(GL_VIEWPORT, viewport_matrix);
        
}

/*******************************************************************************
 Scheduler Callbacks

 These callback routines get performed in the haptics thread
*******************************************************************************/

void MatrixTGldouble(hduMatrix m, GLdouble g[16]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            return;
}
void convert(hduVector3Dd v, float v2[4]) {
    v2[0] = v[0];
    v2[1] = v[1];
    v2[2] = v[2];
    v2[3] = 1.0;
}

void mult_matrix_vector(hduMatrix m, float v[4], float f[4]) {
    for (int i = 0; i < 4; i++)
    {
        
        for (int j = 0; j < 4; j++)
        {
            f[i] += m[i][j] * v[j];
        }
    }
}

/*******************************************************************************
 This is the main haptic thread scheduler callback. It handles start the haptic
 frame and updates the currently applied constraint.
*******************************************************************************/
HDCallbackCode HDCALLBACK HapticDeviceManager::beginUpdateCallback(
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);
    IHapticDevice::IHapticDeviceState* pState =
        pThis->m_pHapticDeviceGT->getCurrentState();

    /* Force the haptic device to update its state. */
    pThis->m_pHapticDeviceHT->beginUpdate(0);

    IHapticDevice::IHapticDeviceState *pCurrentState = 
        pThis->m_pHapticDeviceHT->getCurrentState();
    IHapticDevice::IHapticDeviceState *pLastState = 
        pThis->m_pHapticDeviceHT->getLastState();

    /* Get the position of the device. */
    hduVector3Dd devicePositionLC = pCurrentState->getPosition();
    float speed;
    float numerator, denominator;
    long int index = (long int)pState->getContactData();

    hduMatrix parentTworld(pState->getParentCumulativeTransform());
   
    
    hduVector3Dd pointPositionWC = pCurrentState->getPosition();
    

    hduVector3Dd pointPositionLC;
    parentTworld.multVecMatrix(pointPositionWC, pointPositionLC);
    hduVector3Dd final_prov;

    GLdouble final_vector[3];

    GLdouble zero_vector[3];
    GLdouble uno_vector[3];
    gluProject(double(2) * cos(M_PI / 4), double(2) * sin(M_PI / 4), 0, pThis->view_matrix, pThis->projection_matrix, pThis->viewport_matrix, &zero_vector[0], &zero_vector[1], &zero_vector[2]);
    gluProject(double(2) * cos(M_PI / 4), double(2) * sin(M_PI / 4), 0.2, pThis->view_matrix, pThis->projection_matrix, pThis->viewport_matrix, &uno_vector[0], &uno_vector[1], &uno_vector[2]);
    gluProject(pointPositionLC[0], pointPositionLC[1], pointPositionLC[2], pThis->view_matrix, pThis->projection_matrix, pThis->viewport_matrix, &final_vector[0], &final_vector[1], &final_vector[2] );
    cout << "zero  " << zero_vector[0] << " " << zero_vector[1] <<  " " << zero_vector[2] << endl;
    cout << "uno  " << uno_vector[0] << " " << uno_vector[1] << " " << uno_vector[2] << endl;
   // cout << "final  " << final_vector[0] << " " << final_vector[1] << endl;
    float solucion_final[2];
    solucion_final[0] = ((final_vector[0] - 250) / 149.65) * 1.41421;
    solucion_final[1] = ((final_vector[1] - 250) / 149.65) * 1.41421;

    if (pThis->action_initialized) {
        auto new_time = chrono::steady_clock::now();
        float time = chrono::duration_cast<chrono::nanoseconds>(new_time - pThis->begin_time).count();
        // << time << endl;
        pThis->results << solucion_final[0] << "," << solucion_final[1]  << "," << pThis->counter_accions << "," << pThis->actual_action << "," << time << ", \n";
        if (time > 2000000000) {
            cout << "se ha acabado el tiempo" << endl;
            cout << "\a" << endl;
            cout << pThis->counter_accions << endl;
            pThis->total_is_in_contact = TimePoint::max();
            pThis->m_hManipulationCallback = hdScheduleAsynchronous(actionFinished, pThis,
                RENDER_EFFECT_FORCE_PRIORITY);
        }

    }

    if (pCurrentState->isInContact() && !pThis->action_initialized && index == 0 && pThis->getManipulationStyle() != FREE_MANIPULATION) {
       
        if (pThis->total_is_in_contact == TimePoint::max()) {
            
            pThis->total_is_in_contact = chrono::steady_clock::now();
        }
        else if (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - pThis->total_is_in_contact).count() > 1000) {
            
            pThis->total_is_in_contact = TimePoint::max();
            pThis->m_hManipulationCallback = hdScheduleAsynchronous(actionInitialized, pThis,
                RENDER_EFFECT_FORCE_PRIORITY);
            
        }

    }

    else if (pCurrentState->isInContact() && pThis->action_initialized && index == pThis->actual_action && pThis->getManipulationStyle() != FREE_MANIPULATION) {
        
        if (pThis->total_is_in_contact == TimePoint::max()) {
      
            pThis->total_is_in_contact = chrono::steady_clock::now();
        }
        else if (chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - pThis->total_is_in_contact).count() > 300) {
            pThis->total_is_in_contact = TimePoint::max();
       
            pThis->m_hManipulationCallback = hdScheduleAsynchronous(actionFinished, pThis,
                RENDER_EFFECT_FORCE_PRIORITY);
        }
        
    }
    else
        pThis->total_is_in_contact = TimePoint::max();

        



    /* Update the snap constraint state based on the new position date. */
    ISnapConstraintsAPI *pSnapAPI = pThis->m_pSnapAPI;
    pSnapAPI->updateConstraint(devicePositionLC);

    pCurrentState->setIsInContact(pSnapAPI->getConstraint() != 0);

    if (pCurrentState->isInContact())
    {
        /* If currently in contact, use the freshest contact data. */
        pCurrentState->setContactData(pSnapAPI->getConstraint()->getUserData());        
    }
    else if (pLastState->isInContact())
    {
        /* If was in contact the last frame, use that contact data, since it
           will get reported to the event callbacks. */
        pCurrentState->setContactData(pLastState->getContactData());        
    }
    else
    {
        pCurrentState->setContactData((void *) -1);
    }

    /* Transform result from world coordinates back to device coordinates. */
    hduVector3Dd proxyPositionLC = pSnapAPI->getConstrainedProxy();
    pCurrentState->setProxyPosition(proxyPositionLC);

    hdGetDoublev(HD_CURRENT_TRANSFORM, pCurrentState->getProxyTransform());
    pCurrentState->getProxyTransform()[12] = proxyPositionLC[0];
    pCurrentState->getProxyTransform()[13] = proxyPositionLC[1];
    pCurrentState->getProxyTransform()[14] = proxyPositionLC[2];


    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Compute the constraint force based on the proxy position and add any
 additional effect forces. Submit the force to HD and complete the frame.
*******************************************************************************/
HDCallbackCode HDCALLBACK HapticDeviceManager::endUpdateCallback(
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);

    IHapticDevice::IHapticDeviceState *pCurrentState = 
        pThis->m_pHapticDeviceHT->getCurrentState();

    double kProxyStiffness;
    hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &kProxyStiffness);
    kProxyStiffness = hduMin(0.4, kProxyStiffness);
   
    /* Get the position of the device and the position of the proxy
       in local device coordinates. */
    hduVector3Dd devicePositionLC = pCurrentState->getPosition();
    hduVector3Dd proxyPositionLC = pCurrentState->getProxyPosition();

    /* Compute spring force to attract device to constrained proxy. */
    //hduVector3Dd proxyForce = kProxyStiffness * (proxyPositionLC - devicePositionLC);
    hduVector3Dd proxyForce = hduVector3Dd(0, 0, 0);
    /* Combine proxy force with current effect force. */
    hduVector3Dd totalForce = proxyForce + pThis->m_effectForce;

    hdSetDoublev(HD_CURRENT_FORCE, totalForce);

    pThis->m_pHapticDeviceHT->endUpdate(0);

    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 A scheduler callback for computing an anchored spring effect. The resultant
 force is stored in an effect force data member, which can be combined with
 other forces that are computed during this frame.
*******************************************************************************/
HDCallbackCode HDCALLBACK HapticDeviceManager::springManipulationCallback(
    void *pUserData)
{
    static const double kStiffness = 0.2;

    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);

    IHapticDevice::IHapticDeviceState *pState = 
        pThis->m_pHapticDeviceHT->getCurrentState();

    /* Use the proxy position as input, so that we can apply the spring
       force but still respect the constraint imposed on the proxy. */
    hduVector3Dd devicePositionLC = pState->getProxyPosition();
    
    hduVector3Dd force = kStiffness * (pThis->m_deviceAnchor - devicePositionLC);

    /* Clamp the force using the max continuous force. */
    HDdouble clamp;
    hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &clamp);
    if (force.magnitude() > clamp)
    {
        force.normalize();
        force *= clamp;
    }

    pThis->m_effectForce += force;

    return HD_CALLBACK_CONTINUE;
}

/******************************************************************************
 A scheduler callback for computing a smooth friction effect. This
 force is stored in an effect force data member, which can be combined with
 other forces that are computed during this frame.
******************************************************************************/
HDCallbackCode HDCALLBACK HapticDeviceManager::frictionManipulationCallback(
    void *pUserData)
{
    static const double kDamping = 0.003;
    static const double kClamp = 0.5;

    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);

    hduVector3Dd velocity;
    hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
    
    hduVector3Dd force = -kDamping * velocity;

    /* Clamp the force using the max continuous force. */
    if (force.magnitude() > kClamp)
    {
        force.normalize();
        force *= kClamp;
    }

    pThis->m_effectForce += force;

    return HD_CALLBACK_CONTINUE;
}

/******************************************************************************
 Scheduler callback to set the workspace transform both for use in the graphics
 thread and haptics thread.
******************************************************************************/
HDCallbackCode HDCALLBACK HapticDeviceManager::setDeviceTransformCallback(
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);

    IHapticDevice::IHapticDeviceState *pStateGT = 
        pThis->m_pHapticDeviceGT->getCurrentState();
    IHapticDevice::IHapticDeviceState *pStateHT = 
        pThis->m_pHapticDeviceHT->getCurrentState();

    hduMatrix parentTworld(pThis->m_offsetXform);    
    parentTworld.multRight(pThis->m_workspaceXform);

    pStateGT->setParentCumulativeTransform(parentTworld);
    pStateHT->setParentCumulativeTransform(parentTworld);

    return HD_CALLBACK_DONE;
}

/******************************************************************************
 Scheduler callback to set the current constraint
******************************************************************************/
HDCallbackCode HDCALLBACK HapticDeviceManager::setConstraintCallback(
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);

    pThis->m_pSnapAPI->setConstraint(pThis->m_pConstraint);

    return HD_CALLBACK_DONE;
}
/******************************************************************************
 Scheduler callback to set the current constraint
******************************************************************************/
HDCallbackCode HDCALLBACK HapticDeviceManager::clearConstraintCallback(
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);

    pThis->m_pSnapAPI->clearConstraint();

    return HD_CALLBACK_DONE;
}

hduVector3Dd forceField()
{
   

    hduVector3Dd forceVec(0, 0, 0);

    hduVector3Dd dir_force(-1, 0, 0);
    forceVec = dir_force;

    return forceVec;
}


HDCallbackCode HDCALLBACK HapticDeviceManager::CoulombCallback(void* pUserData)
{

    HapticDeviceManager* pThis = static_cast<HapticDeviceManager*>(pUserData);

    hduVector3Dd forceVec;
    forceVec = forceField();
    pThis->m_effectForce=forceVec;

    return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK HapticDeviceManager::clearForceCallback(void* pUserData)
{
    HapticDeviceManager* pThis = static_cast<HapticDeviceManager*>(pUserData);


    hduVector3Dd forceVec(0,0,0);
    pThis->m_effectForce = forceVec;

    return HD_CALLBACK_CONTINUE;
}

HDCallbackCode HDCALLBACK HapticDeviceManager::actionInitialized(void* pUserData)
{

    HapticDeviceManager* pThis = static_cast<HapticDeviceManager*>(pUserData);
    pThis->action_initialized = true;

   

    pThis->m_pPointManager->setPointSelected(0, true);
    pThis->m_pPointManager->setPointHighlighted(0, false);
  


    if (pThis->point_repetition == 10) {
        pThis->next_point = (pThis->next_point +1) % 8;
        pThis->point_repetition = 0;

    }
    pThis->point_repetition += 1;
    pThis->actual_action = pThis->next_point + 1;
    pThis->actual_action_position = pThis->m_pPointManager->getPointPosition(pThis->next_point + 1);

    pThis->m_pPointManager->setPointHighlighted(pThis->next_point + 1, true);
    

    pThis->m_pPointManager->setPointSelected(pThis->next_point + 1, false);

    if (pThis->getManipulationStyle() == FORCE_MANIPULATION || pThis->getManipulationStyle() == FORCE_MANIPULATION_NOCURSOR)
        pThis->m_hManipulationCallback = hdScheduleAsynchronous(CoulombCallback, pThis,
            RENDER_EFFECT_FORCE_PRIORITY);

  
    pThis->begin_time = chrono::steady_clock::now();

    pThis->counter_accions++;

    if (pThis->getManipulationStyle() == NOFORCE_MANIPULATION_NOCURSOR || pThis->getManipulationStyle() == FORCE_MANIPULATION_NOCURSOR) {
        pThis->red = 0.0;
        pThis->green = 0.0;
        pThis->blue = 0.0;
    }


    return HD_CALLBACK_DONE;
}



HDCallbackCode HDCALLBACK HapticDeviceManager::actionFinished(void* pUserData)
{
    
    HapticDeviceManager* pThis = static_cast<HapticDeviceManager*>(pUserData);
    pThis->action_initialized = false;
  

    pThis->m_pPointManager->setPointSelected(pThis->actual_action, true);
    pThis->m_pPointManager->setPointHighlighted(pThis->actual_action, false);

    pThis->m_pPointManager->setPointHighlighted(0, true);
    pThis->m_pPointManager->setPointSelected(0, false);

    

    pThis->m_hManipulationCallback = hdScheduleAsynchronous(clearForceCallback, pThis,
        RENDER_EFFECT_FORCE_PRIORITY);

    pThis->total_actions++;

    pThis->red = 1.0;
    pThis->green = 1.0;
    pThis->blue = 1.0;

    if (pThis->total_actions == pThis->max_repetitions) {
        pThis->setManipulationStyle(FREE_MANIPULATION);
        pThis->total_actions = 0;
    }

    return HD_CALLBACK_DONE;
}
/******************************************************************************
 Event Callbacks

 These are event callbacks that are registered with the IHapticDevice
******************************************************************************/

/******************************************************************************
 This handler gets called in the graphics thread whenever the device makes
 contact with a constraint. Provide a visual cue (i.e. highlighting) to
 accompany the haptic cue of being snapped to the point.
******************************************************************************/
void HapticDeviceManager::madeContactCallbackGT(
    IHapticDevice::EventType event,
    const IHapticDevice::IHapticDeviceState * const pState,
    void *pUserData)
{
   
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData); 

    
        /* Get the index of the contacted point from the contact data. */
        long int index = (long int)pState->getContactData();
   
        if (pThis->getManipulationStyle() != FREE_MANIPULATION) {

            if (pThis->getManipulationStyle() != FREE_MANIPULATION) {
                if (index == 0 && (!pThis->action_initialized)) {

                 
                }


            }

            if (index != 0 && (index == pThis->actual_action) && pThis->action_initialized)
            {
                
            
               
             
            }

        }
    
    
   
}

/******************************************************************************
 This handler gets called in the graphics thread whenever the device loses
 contact with a constraint. Provide a visual cue (i.e. highlighting) to
 accompany the haptic cue of losing contact with the point.
******************************************************************************/
void HapticDeviceManager::lostContactCallbackGT(
    IHapticDevice::EventType event,
    const IHapticDevice::IHapticDeviceState * const pState,
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);     
    

    /* Get the index of the contacted point from the contact data. */
    long int index = (long int) pState->getContactData();    


    
}

/******************************************************************************
 This handler gets called in the graphics thread whenever a button press is
 detected. Interpret the click as a select/deselect operation.
******************************************************************************/
void HapticDeviceManager::buttonDownCallbackGT(
    IHapticDevice::EventType event,
    const IHapticDevice::IHapticDeviceState * const pState,
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);     

    pThis->valid = false;
    pThis->red = 1.0;
    pThis->green = 1.0;
    pThis->blue = 1.0;
   
 
        /* Get the index of the contacted point from the contact data. */
        long int index = (long int) pState->getContactData();    

       

   
}

/******************************************************************************
 This handler gets called in the graphics thread in response to a button up
 event. Respond to the button up by stop the current manipulation.
******************************************************************************/
void HapticDeviceManager::buttonUpCallbackGT(
    IHapticDevice::EventType event,
    const IHapticDevice::IHapticDeviceState * const pState,
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);     

    /* Initiate manipulation with this selected point. */
    pThis->stopManipulation();
}


/******************************************************************************
 This handler gets called in the haptics thread in response to a button down
 event. Respond to the button down by initializing the manipulation force
 feedback.
******************************************************************************/
void HapticDeviceManager::buttonDownCallbackHT(
    IHapticDevice::EventType event,
    const IHapticDevice::IHapticDeviceState * const pState,
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);     
    
    if (pState->isInContact())
    {
        /* Determine an offset that will move the local coordinate position
           of the device to the snap point location. */
        hduVector3Dd snapPointLC = pThis->m_pConstraint->
            getPointConstraint()->getPoint();
        
        pThis->m_offsetXform = hduMatrix::createTranslation(
            snapPointLC - pState->getPosition());
        hdScheduleSynchronous(setDeviceTransformCallback, pThis,
                              SYNCHRONIZE_STATE_PRIORITY);    
        
        /* Clear the current constraint and allow for free manipulation. */
        pThis->m_pSnapAPI->clearConstraint();
       
    }
}

/******************************************************************************
 This handler gets called in the haptic thread in response to a button up
 event. Handle this event by stop any force feedback enabled during the 
 manipulation.
******************************************************************************/
void HapticDeviceManager::buttonUpCallbackHT(
    IHapticDevice::EventType event,
    const IHapticDevice::IHapticDeviceState * const pState,
    void *pUserData)
{
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);     

    if (pThis->m_hManipulationCallback != HD_INVALID_HANDLE)
    {
        hdUnschedule(pThis->m_hManipulationCallback);
        pThis->m_hManipulationCallback = HD_INVALID_HANDLE;
    }

    /* Remove the offset transform */
    pThis->m_offsetXform.makeIdentity();
    hdScheduleSynchronous(setDeviceTransformCallback, pThis,
                          SYNCHRONIZE_STATE_PRIORITY);

    
    /* Set an anti-constraint at the current constraint device location
       This will ensure that the device isn't immediately snapped to something
       that it happens to be nearby. */
    PointConstraint *pAntiConstraint = new PointConstraint(pState->getPosition());
    pAntiConstraint->setAntiConstraint(true);
    pAntiConstraint->setUserData((void *) -1); /* invalid point index. */
    pAntiConstraint->setSnapDistance(SnapConstraint::getDefaultSnapDistance() * 2);
    pThis->m_pSnapAPI->setConstraint(pAntiConstraint);
}

/******************************************************************************
 This handler gets called to handle errors
******************************************************************************/
void HapticDeviceManager::errorCallbackGT(
    IHapticDevice::EventType event,
    const IHapticDevice::IHapticDeviceState * const pState,
    void *pUserData)
{   
    HapticDeviceManager *pThis = static_cast<HapticDeviceManager *>(pUserData);

    if (hduIsForceError(&pState->getLastError()))
    {
        /* Clear the current constraint. */
        hdScheduleSynchronous(clearConstraintCallback, pThis,
            SYNCHRONIZE_STATE_PRIORITY);
    }
    else
    {
        /* This is likely a more serious error, so just bail. */
        std::cerr << pState->getLastError() << std::endl;
        std::cerr << "Error during haptic rendering" << std::endl;
        std::cerr << "Press any key to quit." << std::endl;
        getchar();
        exit(-1);
    }
}

/******************************************************************************/
