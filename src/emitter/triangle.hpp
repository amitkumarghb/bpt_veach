#pragma once

#include <cmath>
#include <tuple>

#include "../emitter/polymorphic.hpp"

#include "../colour/colour.hpp"
#include "../mathematics/double3.hpp"
#include "../mathematics/orthogonal.hpp"
#include "../random/mersenne.hpp"
#include "../sample/hemisphere.hpp"
#include "../sample/triangle.hpp"

namespace Emitter
{

	class Triangle final : public Emitter::Polymorphic
	{

	private:

		Double3 position; // a
		Double3 edge1; // b-a
		Double3 edge2; // c-a
		Double3 normal; // edge1 cross edge2

		Orthogonal local_space;

		Colour energy;

		std::float_t pdf_area;

	public:

		Triangle() = delete;

		Triangle(
			Double3 const& a,
			Double3 const& b,
			Double3 const& c,
			Colour const& energy
		) :
			position( a ), edge1( b - a ), edge2( c - a ), energy( energy )
		{
			Double3 const cross_product = edge1.cross( edge2 );
			normal = ( cross_product ).normalise();
			local_space = Orthogonal( normal );
			pdf_area = 1. / ( .5 * ( cross_product ).magnitude() );
		};

		std::tuple <Colour, Double3, Double3, Double3, std::float_t, std::float_t, std::float_t> emit(
			Random::Mersenne& prng
		) const override
		{
			auto const [u, v] = Sample::Triangle( prng );
			Double3 const point = position + edge1 * u + edge2 * v;
			Double3 const local_sample = Sample::HemiSphere( prng );
			Double3 const direction = local_space.to_world( local_sample );
			return { energy, point, direction, normal, local_sample.z * inv_pi, pdf_area, local_sample.z };
		};

		Colour radiance(
			Double3 const& eval_point,
			Double3 const& eval_direction
		) const override
		{
			std::double_t const cos_theta = normal.dot( eval_direction );
			return cos_theta > 0. ? energy : Colour::Black;
		};

		virtual std::tuple< std::float_t, std::float_t, std::float_t> pdf_Le(
			Double3 const& eval_point,
			Double3 const& eval_direction
		) const override
		{
			std::float_t const cos_theta = normal.dot( eval_direction );
			if ( cos_theta < EPSILON_COS_THETA )
				return { 0.f, 0.f, 0.f };
			return { inv_2pi, pdf_area, cos_theta };
		};

		std::float_t pdf_W(
			Double3 const&,
			Double3 const& eval_direction
		) const override {
			std::float_t const cos_theta = normal.dot( eval_direction );
			if ( cos_theta < EPSILON_COS_THETA )
				return 0.f;
			return cos_theta * inv_pi;
		};

		std::float_t pdf_A(
			Double3 const&,
			Double3 const&
		) const override {
			return pdf_area;
		};

		Emitter::Type type() const override { return Emitter::Type::Area; };

		bool is_dirac() const override { return false; };

	};

};

