/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file MockLink.h
 * @brief This file contains the header of the MockLink class.
 *
 * @author Pedro Otávio Fonseca Pires
 */

#ifndef PROVANT_MOCK_LINK_H
#define PROVANT_MOCK_LINK_H

#include <gazebo/physics/Link.hh>
#include <gmock/gmock.h>

namespace provant::tests::gz
{
class MockLink : public gazebo::physics::Link
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
  //---------- Virtual methods of the Entity class ----------

  //---------- Overloads from the Base class ----------

  // virtual void Load(sdf::ElementPtr _sdf);
  // virtual void Fini();
  // virtual void Reset();
  // virtual void UpdateParameters(sdf::ElementPtr _sdf);
  // virtual void SetName(const std::string &_name);

  //---------- Original virtual methods of the Entity class ----------

  // virtual ignition::math::AxisAlignedBox BoundingBox() const;
  MOCK_CONST_METHOD0(BoundingBox, ignition::math::AxisAlignedBox());
  // virtual ignition::math::Vector3d RelativeLinearVel() const;
  MOCK_CONST_METHOD0(RelativeLinearVel, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d WorldLinearVel() const;
  MOCK_CONST_METHOD0(WorldLinearVel, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d RelativeAngularVel() const;
  MOCK_CONST_METHOD0(RelativeAngularVel, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d WorldAngularVel() const;
  MOCK_CONST_METHOD0(WorldAngularVel, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d RelativeLinearAccel() const;
  MOCK_CONST_METHOD0(RelativeLinearAccel, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d WorldLinearAccel() const;
  MOCK_CONST_METHOD0(WorldLinearAccel, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d RelativeAngularAccel() const;
  MOCK_CONST_METHOD0(RelativeAngularAccel, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d WorldAngularAccel() const;
  MOCK_CONST_METHOD0(WorldAngularAccel, ignition::math::Vector3d());
  // virtual void StopAnimation();
  MOCK_METHOD0(StopAnimation, void());
  // virtual void OnPoseChange() = 0;
  MOCK_METHOD0(OnPoseChange, void());
  // virtual void PublishPose();
  MOCK_METHOD0(PublishPose, void());

  //---------------------------------------------------------------------------------------------------------------------------
  //---------- Virtual methods of the Link class ----------

  //---------- Overloads from the Base class ----------

  // virtual void Load(sdf::ElementPtr _sdf) override;
  // virtual void Init() override;
  // virtual void UpdateParameters(sdf::ElementPtr _sdf) override;
  // virtual void RegisterIntrospectionItems() override;
  // virtual bool SetSelected(bool _set) override;

  //---------- Overloads from the Entity class ----------

  // virtual ignition::math::AxisAlignedBox BoundingBox() const override;
  // virtual ignition::math::Vector3d WorldLinearVel() const override;
  // virtual void OnPoseChange() override;

  //---------- Original virtual methods of the Link class ----------

  // virtual void SetEnabled(bool _enable) const = 0;
  MOCK_CONST_METHOD1(SetEnabled, void(bool _enable));
  // virtual bool GetEnabled() const = 0;
  MOCK_CONST_METHOD0(GetEnabled, bool());
  // virtual void SetGravityMode(bool _mode) = 0;
  MOCK_METHOD1(SetGravityMode, void(bool _mode));
  // virtual bool GetGravityMode() const = 0;
  MOCK_CONST_METHOD0(GetGravityMode, bool());
  // virtual void SetWindMode(const bool _mode);
  MOCK_METHOD1(SetWindMode, void(const bool _mode));
  // virtual bool WindMode() const;
  MOCK_CONST_METHOD0(WindMode, bool());
  // virtual void SetSelfCollide(bool _collide) = 0;
  MOCK_METHOD1(SetSelfCollide, void(bool _collide));
  // virtual void SetLinearVel(const ignition::math::Vector3d &_vel) = 0;
  MOCK_METHOD1(SetLinearVel, void(const ignition::math::Vector3d& _vel));
  // virtual void SetAngularVel(const ignition::math::Vector3d &_vel) = 0;
  MOCK_METHOD1(SetAngularVel, void(const ignition::math::Vector3d& _vel));
  // virtual void SetForce(const ignition::math::Vector3d &_force) = 0;
  MOCK_METHOD1(SetForce, void(const ignition::math::Vector3d& _force));
  // virtual void SetTorque(const ignition::math::Vector3d &_torque) = 0;
  MOCK_METHOD1(SetTorque, void(const ignition::math::Vector3d& _torque));
  // virtual void AddForce(const ignition::math::Vector3d &_force) = 0;
  MOCK_METHOD1(AddForce, void(const ignition::math::Vector3d& _force));
  // virtual void AddRelativeForce(const ignition::math::Vector3d &_force) = 0;
  MOCK_METHOD1(AddRelativeForce, void(const ignition::math::Vector3d& _force));
  // virtual void AddForceAtWorldPosition(const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_pos)
  // = 0;
  MOCK_METHOD2(AddForceAtWorldPosition,
               void(const ignition::math::Vector3d& _force, const ignition::math::Vector3d& _pos));
  // virtual void AddForceAtRelativePosition(const ignition::math::Vector3d &_force, const ignition::math::Vector3d
  // &_relPos) = 0;
  MOCK_METHOD2(AddForceAtRelativePosition,
               void(const ignition::math::Vector3d& _force, const ignition::math::Vector3d& _relPos));
  // virtual void AddLinkForce(const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_offset) = 0;
  MOCK_METHOD2(AddLinkForce, void(const ignition::math::Vector3d& _force, const ignition::math::Vector3d& _offset));
  // virtual void AddTorque(const ignition::math::Vector3d &_torque) = 0;
  MOCK_METHOD1(AddTorque, void(const ignition::math::Vector3d& _torque));
  // virtual void AddRelativeTorque(const ignition::math::Vector3d &_torque) = 0;
  MOCK_METHOD1(AddRelativeTorque, void(const ignition::math::Vector3d& _force));
  // virtual ignition::math::Vector3d WorldLinearVel(const ignition::math::Vector3d &_offset) const = 0;
  MOCK_CONST_METHOD1(WorldLinearVel, ignition::math::Vector3d(const ignition::math::Vector3d& _offset));
  // virtual ignition::math::Vector3d WorldLinearVel(const ignition::math::Vector3d &_offset, const
  // ignition::math::Quaterniond &_q) const = 0;
  MOCK_CONST_METHOD2(WorldLinearVel, ignition::math::Vector3d(const ignition::math::Vector3d& _offset,
                                                              const ignition::math::Quaterniond& _q));
  // virtual ignition::math::Vector3d WorldCoGLinearVel() const = 0;
  MOCK_CONST_METHOD0(WorldCoGLinearVel, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d WorldForce() const = 0;
  MOCK_CONST_METHOD0(WorldForce, ignition::math::Vector3d());
  // virtual ignition::math::Vector3d WorldTorque() const = 0;
  MOCK_CONST_METHOD0(WorldTorque, ignition::math::Vector3d());
  // virtual void SetLinearDamping(double _damping) = 0;
  MOCK_METHOD1(SetLinearDamping, void(double _damping));
  // virtual void SetAngularDamping(double _damping) = 0;
  MOCK_METHOD1(SetAngularDamping, void(double _damping));
  // virtual void SetKinematic(const bool &_kinematic);
  MOCK_METHOD1(SetKinematic, void(const bool& _kinematic));
  // virtual bool GetKinematic() const
  MOCK_CONST_METHOD0(GetKinematic, bool());
  // virtual void RemoveChild(EntityPtr _child);
  MOCK_METHOD1(RemoveChild, void(gazebo::physics::EntityPtr _child));
  // virtual void UpdateMass()
  MOCK_METHOD0(UpdateMass, void());
  // virtual void UpdateSurface()
  MOCK_METHOD0(UpdateSurface, void());
  // virtual void SetAutoDisable(bool _disable) = 0;
  MOCK_METHOD1(SetAutoDisable, void(bool _disable));
  // virtual void SetLinkStatic(bool _static) = 0;
  MOCK_METHOD1(SetLinkStatic, void(bool _static));
  // virtual void SetStatic(const bool &_static);
  MOCK_METHOD1(SetStatic, void(const bool& _static));
};
}  // namespace provant::tests::gz

#endif  // PROVANT_MOCK_LINK_H
