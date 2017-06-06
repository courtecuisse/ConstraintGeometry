#ifndef SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_INL
#define SOFA_COMPONENT_CONSTRAINT_CONSTRAINTGEOMETRY_INL

#include "ConstraintGeometry.h"
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {

BaseGeometry * BaseDecorator::getGeometry() {
    BaseGeometry * geo = NULL;

    this->getContext()->get(geo, core::objectmodel::BaseContext::Local);

    if (geo == NULL) serr << "Error cannot find the geometry" << sendl;

    return geo;
}

void BaseGeometry::init() {
    this->getContext()->get(m_topology);
    this->getContext()->get(m_state);

    if (getTopology() == NULL) serr << "Error cannot find the topology" << sendl;
    if (getMstate() == NULL) serr << "Error cannot find the topology" << sendl;
}

std::unique_ptr<BaseConstraintIterator> BaseGeometry::getIterator(const ConstraintProximity & P) {
    BaseDecorator * decorator;
    this->getContext()->get(decorator);

    if (decorator != NULL) return decorator->getIterator(P);
    std::unique_ptr<BaseConstraintIterator> foo (new DefaultConstraintIterator(this));
    return foo;
}

void BaseGeometry::addConstraint(core::MultiMatrixDerivId cId,unsigned cline,const ConstraintProximity & pinfo,const defaulttype::Vector3 & normal) {
    DataMatrixDeriv & c_d = *cId[getMstate()].write();

    MatrixDeriv & c = *c_d.beginEdit();

    MatrixDerivRowIterator c_it1 = c.writeLine(cline);

    for (unsigned p=0;p<pinfo.size();p++) {
        if (pinfo.m_fact[p] == 0.0) continue;
        c_it1.addCol(pinfo.m_pid[p], normal * pinfo.m_fact[p]);
    }

    c_d.endEdit();
}

//linear version of getPosition
defaulttype::Vector3 BaseGeometry::getPosition(const ConstraintProximity & pinfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::position());

    defaulttype::Vector3 P;
    for (unsigned i=0;i<pinfo.size();i++) {
        P += x[pinfo.m_pid[i]] * pinfo.m_fact[i];
    }
    return P;
}

defaulttype::Vector3 BaseGeometry::getFreePosition(const ConstraintProximity & pinfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::freePosition());

    defaulttype::Vector3 P;
    for (unsigned i=0;i<pinfo.size();i++) {
        P += x[pinfo.m_pid[i]] * pinfo.m_fact[i];
    }
    return P;
}

defaulttype::Vector3 BaseGeometry::getRestPosition(const ConstraintProximity & pinfo) {
    const helper::ReadAccessor<Data <VecCoord> >& x = *this->getMstate()->read(core::VecCoordId::restPosition());

    defaulttype::Vector3 P;
    for (unsigned i=0;i<pinfo.size();i++) {
        P += x[pinfo.m_pid[i]] * pinfo.m_fact[i];
    }
    return P;
}


} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
