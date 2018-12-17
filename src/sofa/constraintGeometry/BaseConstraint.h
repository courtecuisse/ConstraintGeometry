#pragma once

#include <sofa/collisionAlgorithm/BaseCollisionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/behavior/BaseConstraint.h>
#include <sofa/constraintGeometry/BaseResponse.h>

namespace sofa {

namespace constraintGeometry {

class BaseConstraint : public sofa::core::behavior::BaseConstraint {
public:
    SOFA_CLASS(BaseConstraint, sofa::core::behavior::BaseConstraint);

    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Real Real;
    typedef DataTypes::VecDeriv VecDeriv;
    typedef DataTypes::MatrixDeriv MatrixDeriv;
    typedef DataTypes::Deriv Deriv1;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef MatrixDeriv::RowIterator MatrixDerivRowIterator;

    Data<double> d_drawScale;
    Data<defaulttype::Vector4> d_drawColor;

    BaseConstraint()
    : d_drawScale(initData(&d_drawScale, 1.0, "draw_scale", "draw scale"))
    , d_drawColor(initData(&d_drawColor, defaulttype::Vector4(1,0,0,1), "draw_color", "draw color"))
    , l_detection(initLink("response", "Link to Response")) {}

    virtual ConstraintReponse * createResponse(const collisionAlgorithm::DetectionOutput::SPtr d) = 0;

    void processGeometricalData() {
        //each component of this vector will be deleted by sofa at each time step so we don't have to delete each component
        m_constraints.clear();

        for (unsigned i=0;i<l_detection->getNbOutput();i++) {
            m_constraints.push_back(createResponse(l_detection->getOutput(i)));
        }
    }

    void buildConstraintMatrix(const core::ConstraintParams* /*cParams*/, core::MultiMatrixDerivId cId, unsigned int &constraintId) {
        m_cid = constraintId;

        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->buildConstraintMatrix(cId,constraintId);
            constraintId += m_constraints[i]->size();
        }
    }

    void getConstraintViolation(const core::ConstraintParams* /*cParams*/, defaulttype::BaseVector *v,unsigned cid) {
        cid = m_cid;

        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->getConstraintViolation(v, cid);
            cid += m_constraints[i]->size();
        }
    }

    void getConstraintResolution(const core::ConstraintParams* /*cParams*/, std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset) {
        for (unsigned i=0;i<m_constraints.size();i++) {
            resTab[offset] = m_constraints[i];
            offset+=m_constraints[i]->size();
        }
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowInteractionForceFields()) return;

        for (unsigned i=0;i<m_constraints.size();i++) {
            m_constraints[i]->draw(vparams,d_drawScale.getValue(),d_drawColor.getValue());
        }
    }

//    void storeLambda(const core::ConstraintParams* cParams, Data<VecDeriv>& result, const Data<MatrixDeriv>& jacobian, const sofa::defaulttype::BaseVector* lambda) {
//        auto res = sofa::helper::write(result, cParams);
//        const MatrixDeriv& j = jacobian.getValue(cParams);
//        j.multTransposeBaseVector(res, lambda ); // lambda is a vector of scalar value so block size is one.
//    }

    void storeLambda(const core::ConstraintParams* /*cParams*/, core::MultiVecDerivId /*res*/, const sofa::defaulttype::BaseVector* /*lambda*/) {
//        if (cParams) {
//            for (auto it=m_state.cbegin();it!=m_state.cend();it++) {
//                storeLambda(cParams, *res[(*it)].write(), *cParams->readJ((*it)), lambda);
//            }
//        }
    }

    void updateForceMask() {}

protected:    
    unsigned m_cid;
    std::vector<ConstraintReponse *> m_constraints;
    core::objectmodel::SingleLink<BaseConstraint,collisionAlgorithm::BaseCollisionAlgorithm,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_detection;
};

}

}