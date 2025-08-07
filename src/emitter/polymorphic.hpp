#pragma once

#include <tuple>

#include "../colour/colour.hpp"
#include "../mathematics/double3.hpp"
#include "../random/mersenne.hpp"

namespace Emitter
{

	enum class Type
	{
		Area,
		Directional,
		Environment,
		Point,
		Spot
	};

	class Polymorphic
	{

	public:

		// Returns:
		// Energy, point on emitter, direction from emitter,
		// emitter normal at point (not all emitters have this, see is_dirac()),
		// pdf_W, pdf_A, cos_theta
		virtual std::tuple <Colour, Double3, Double3, Double3, std::float_t, std::float_t, std::float_t> emit(
			Random::Mersenne& prng
		) const = 0;

		virtual Colour radiance(
			Double3 const& eval_point,
			Double3 const& eval_direction // Direction is away from emitter/eval point
		) const = 0;

		// Evaluate a point on an emitter
		// pdf_W, pdf_A, cos_theta
		virtual std::tuple< std::float_t, std::float_t, std::float_t> pdf_Le(
			Double3 const& eval_point,
			Double3 const& eval_direction // Direction is away from emitter/eval point
		) const = 0;

		// Evaluate a point on an emitter
		// pdf_W, pdf_A, cos_theta
		virtual std::float_t pdf_W(
			Double3 const& eval_point,
			Double3 const& eval_direction // Direction is away from emitter/eval point
		) const = 0;

		virtual std::float_t pdf_A(
			Double3 const& eval_point,
			Double3 const& eval_direction // Direction is away from emitter/eval point
		) const = 0;

		virtual Emitter::Type type() const = 0;

		// True for emitters than can not be intersected (point/directional)
		virtual bool is_dirac() const = 0;

	};

};
