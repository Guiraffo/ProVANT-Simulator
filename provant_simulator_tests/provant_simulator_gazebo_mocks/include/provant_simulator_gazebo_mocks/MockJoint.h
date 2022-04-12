/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file MockJoint.h
 * @brief This file contains the header of the MockJoint class.
 *
 * @author Pedro Otávio Fonseca Pires
 */

#ifndef PROVANT_MOCK_JOINT_H
#define PROVANT_MOCK_JOINT_H

#include <gazebo/physics/Joint.hh>
#include <gmock/gmock.h>

namespace provant::tests::gz
{
class MockJoint : public gazebo::physics::Joint
{
public:
  //---------- Virtual methods of the Base class ----------

  // virtual void Load(sdf::ElementPtr _sdf);
  MOCK_METHOD1(Load, void(sdf::ElementPtr _sdf));
  // virtual void Fini();
  MOCK_METHOD0(Fini, void());
  // virtual void Init()
  MOCK_METHOD0(Init, void());
  // virtual void Reset();
  MOCK_METHOD0(Reset, void());
  // virtual void Reset(Base::EntityType _resetType);
  MOCK_METHOD1(Reset, void(Base::EntityType _resetType));
  // virtual void Update()
  MOCK_METHOD0(Update, void());
  // virtual void UpdateParameters(sdf::ElementPtr _sdf);
  MOCK_METHOD1(UpdateParameters, void(sdf::ElementPtr _sdf));
  // virtual void SetName(const std::string &_name);
  MOCK_METHOD1(SetName, void(const std::string& _name));
  // virtual void RemoveChild(unsigned int _id);
  MOCK_METHOD1(RemoveChild, void(unsigned int _id));
  // virtual bool SetSelected(bool _show);
  MOCK_METHOD1(SetSelected, bool(bool _show));
  // virtual const sdf::ElementPtr GetSDF();
  MOCK_METHOD0(GetSDF, const sdf::ElementPtr());
  // virtual std::optional<sdf::SemanticPose> SDFSemanticPose() const;
  MOCK_CONST_METHOD0(SDFSemanticPose, std::optional<sdf::SemanticPose>());
  // virtual void RegisterIntrospectionItems();
  MOCK_METHOD0(RegisterIntrospectionItems, void());
  // virtual void UnregisterIntrospectionItems();
  MOCK_METHOD0(UnregisterIntrospectionItems, void());

  //---------------------------------------------------------------------------------------------------------------------------
  //---------- Virtual methods of the Joint class ----------

  //---------- Overloads from the Base class ----------

  // virtual void Load(sdf::ElementPtr _sdf) override;
  // virtual void Init() override;
  // virtual void Fini() override;
  // virtual void UpdateParameters(sdf::ElementPtr _sdf) override;
  // virtual void Reset() override;
  // virtual std::optional<sdf::SemanticPose> SDFSemanticPose() const override;
  // virtual void RegisterIntrospectionItems() override;

  //---------- Original virtual methods of the Joint class ----------

  // virtual LinkPtr GetJointLink(unsigned int _index) const = 0;
  MOCK_CONST_METHOD1(GetJointLink, gazebo::physics::LinkPtr(unsigned int _index));
  // virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const = 0;
  MOCK_CONST_METHOD2(AreConnected, bool(gazebo::physics::LinkPtr _one, gazebo::physics::LinkPtr _two));
  // virtual void Attach(LinkPtr _parent, LinkPtr _child);
  MOCK_METHOD2(Attach, void(gazebo::physics::LinkPtr _parent, gazebo::physics::LinkPtr _child));
  // virtual void Detach();
  MOCK_METHOD0(Detach, void());
  // virtual void SetAxis(const unsigned int _index, const ignition::math::Vector3d &_axis) = 0;
  MOCK_METHOD2(SetAxis, void(const unsigned int _index, const ignition::math::Vector3d& _axis));
  // virtual void SetDamping(unsigned int _index, double _damping) = 0;
  MOCK_METHOD2(SetDamping, void(unsigned int _index, double _damping));
  // virtual void ApplyStiffnessDamping();
  MOCK_METHOD0(ApplyStiffnessDamping, void());
  // virtual void SetStiffnessDamping(unsigned int _index, double _stiffness, double _damping, double _reference = 0) =
  // 0;
  MOCK_METHOD4(SetStiffnessDamping, void(unsigned int _index, double _stiffness, double _damping, double _reference));
  // virtual void SetStiffness(unsigned int _index, const double _stiffness) = 0;
  MOCK_METHOD2(SetStiffness, void(unsigned int _index, const double _stiffness));
  // virtual ignition::math::Vector3d GlobalAxis(unsigned int _index) const = 0;
  MOCK_CONST_METHOD1(GlobalAxis, ignition::math::Vector3d(unsigned int _index));
  // virtual void SetAnchor(const unsigned int _index, const ignition::math::Vector3d &_anchor) = 0;
  MOCK_METHOD2(SetAnchor, void(const unsigned int _index, const ignition::math::Vector3d& _anchor));
  // virtual ignition::math::Vector3d Anchor(const unsigned int _index) const = 0;
  MOCK_CONST_METHOD1(Anchor, ignition::math::Vector3d(const unsigned int _index));
  // virtual double GetEffortLimit(unsigned int _index);
  MOCK_METHOD1(GetEffortLimit, double(unsigned int _index));
  // virtual void SetEffortLimit(unsigned int _index, double _effort);
  MOCK_METHOD2(SetEffortLimit, void(unsigned int _index, double _effort));
  // virtual double GetVelocityLimit(unsigned int _index);
  MOCK_METHOD1(GetVelocityLimit, double(unsigned int _index));
  // virtual void SetVelocityLimit(unsigned int _index, double _velocity);
  MOCK_METHOD2(SetVelocityLimit, void(unsigned int _index, double _velocity));
  // virtual void SetVelocity(unsigned int _index, double _vel) = 0;
  MOCK_METHOD2(SetVelocity, void(unsigned int _index, double _vel));
  // virtual double GetVelocity(unsigned int _index) const = 0;
  MOCK_CONST_METHOD1(GetVelocity, double(unsigned int _index));
  // virtual void SetForce(unsigned int _index, double _effort) = 0;
  MOCK_METHOD2(SetForce, void(unsigned int _index, double _effort));
  // virtual double GetForce(unsigned int _index);
  MOCK_METHOD1(GetForce, double(unsigned int _index));
  // virtual JointWrench GetForceTorque(unsigned int _index) = 0;
  MOCK_METHOD1(GetForceTorque, gazebo::physics::JointWrench(unsigned int _index));

  /**
   * @brief The "Position" method has the "final" modificator, so it can't be Mocked.
   * @todo find a form to "Mock" this method.
   */
  // virtual double Position(const unsigned int _index = 0) const final;
  // MOCK_CONST_METHOD1(Position, double(const unsigned int _index));

  // virtual unsigned int DOF() const = 0;
  MOCK_CONST_METHOD0(DOF, unsigned int());
  // virtual bool SetPosition(const unsigned int _index, const double _position, const bool _preserveWorldVelocity =
  // false);
  MOCK_METHOD3(SetPosition, bool(const unsigned int _index, const double _position, const bool _preserveWorldVelocity));
  // virtual ignition::math::Vector3d LinkForce(const unsigned int _index) const = 0;
  MOCK_CONST_METHOD1(LinkForce, ignition::math::Vector3d(const unsigned int _index));
  // virtual ignition::math::Vector3d LinkTorque(const unsigned int _index) const = 0;
  MOCK_CONST_METHOD1(LinkTorque, ignition::math::Vector3d(const unsigned int _index));
  // virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value) = 0;
  MOCK_METHOD3(SetParam, bool(const std::string& _key, unsigned int _index, const boost::any& _value));
  // virtual double GetParam(const std::string &_key, unsigned int _index);
  MOCK_METHOD2(GetParam, double(const std::string& _key, unsigned int _index));
  // virtual void FillMsg(msgs::Joint &_msg);
  MOCK_METHOD1(FillMsg, void(gazebo::msgs::Joint& _msg));
  // virtual double LowerLimit(unsigned int _index = 0) const;
  MOCK_CONST_METHOD1(LowerLimit, double(unsigned int _index));
  // virtual double UpperLimit(const unsigned int _index = 0) const;
  MOCK_CONST_METHOD1(UpperLimit, double(const unsigned int _index));
  // virtual void SetLowerLimit(const unsigned int _index, const double _limit);
  MOCK_METHOD2(SetLowerLimit, void(const unsigned int _index, const double _limit));
  // virtual void SetUpperLimit(const unsigned int _index, const double _limit);
  MOCK_METHOD2(SetUpperLimit, void(const unsigned int _index, const double _limit));
  // virtual void SetProvideFeedback(bool _enable);
  MOCK_METHOD1(SetProvideFeedback, void(bool _enable));
  // virtual void CacheForceTorque();
  MOCK_METHOD0(CacheForceTorque, void());
  // virtual double PositionImpl(const unsigned int _index = 0) const = 0;
  MOCK_CONST_METHOD1(PositionImpl, double(const unsigned int _index));
};
}  // namespace provant::tests::gz

#endif  // PROVANT_MOCK_JOINT_H