/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef IGNITION_PHYSICS_CAPSULESHAPE_HH_
#define IGNITION_PHYSICS_CAPSULESHAPE_HH_

#include <string>

#include <ignition/physics/DeclareShapeType.hh>
#include <ignition/physics/Geometry.hh>

namespace ignition
{
  namespace physics
  {
    IGN_PHYSICS_DECLARE_SHAPE_TYPE(CapsuleShape)

    class IGNITION_PHYSICS_VISIBLE GetCapsuleShapeProperties
        : public virtual FeatureWithRequirements<CapsuleShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class CapsuleShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Get the radius of this CapsuleShape
        /// \return the radius of this CapsuleShape
        public: Scalar GetRadius() const;

        /// \brief Get the height (length along the local z-axis) of this
        /// CapsuleShape.
        /// \return the height of this CapsuleShape
        public: Scalar GetHeight() const;
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual Scalar GetCapsuleShapeRadius(
            const Identity &_capsuleID) const = 0;

        public: virtual Scalar GetCapsuleShapeHeight(
            const Identity &_capsuleID) const = 0;
      };
    };

    class IGNITION_PHYSICS_VISIBLE SetCapsuleShapeProperties
        : public virtual FeatureWithRequirements<CapsuleShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class CapsuleShape : public virtual Entity<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        /// \brief Set the radius of this CapsuleShape
        /// \param[in] _radius
        ///   The desired radius of this CapsuleShape
        public: void SetRadius(Scalar _radius);

        /// \brief Set the height of this CapsuleShape
        /// \param[in] _height
        ///   The desired height of this CapsuleShape
        public: void SetHeight(Scalar _height);
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: virtual void SetCapsuleShapeRadius(
            const Identity &_capsuleID, Scalar _radius) = 0;

        public: virtual void SetCapsuleShapeHeight(
            const Identity &_capsuleID, Scalar _height) = 0;
      };
    };

    class IGNITION_PHYSICS_VISIBLE AttachCapsuleShapeFeature
        : public virtual FeatureWithRequirements<CapsuleShapeCast>
    {
      public: template <typename PolicyT, typename FeaturesT>
      class Link : public virtual Feature::Link<PolicyT, FeaturesT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: using ShapePtrType = CapsuleShapePtr<PolicyT, FeaturesT>;

        public: ShapePtrType AttachCapsuleShape(
            const std::string &_name = "capsule",
            Scalar _radius = 1.0,
            Scalar _height = 1.0,
            const PoseType &_pose = PoseType::Identity());
      };

      public: template <typename PolicyT>
      class Implementation : public virtual Feature::Implementation<PolicyT>
      {
        public: using Scalar = typename PolicyT::Scalar;

        public: using PoseType =
            typename FromPolicy<PolicyT>::template Use<Pose>;

        public: virtual Identity AttachCapsuleShape(
            const Identity &_linkID,
            const std::string &_name,
            Scalar _radius,
            Scalar _height,
            const PoseType &_pose) = 0;
      };
    };
  }
}

#include <ignition/physics/detail/CapsuleShape.hh>

#endif
