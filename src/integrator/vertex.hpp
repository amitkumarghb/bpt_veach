#pragma once

#include "../bxdf/polymorphic.hpp"
#include "../colour/colour.hpp"
#include "../emitter/polymorphic.hpp"
#include "../mathematics/double3.hpp"
#include "../ray/intersection.hpp"

namespace Integrator
{

	// Storage for path traced vertices
	struct Vertex
	{

		Ray::Intersection idata;

		Colour throughput;

		std::float_t pdf_forward;
		std::float_t pdf_reverse;

		std::double_t G{ 1. };

		bool f_dirac;
		bool f_emitter;
		// Note: only dirac camera is implemented
		bool f_camera;

		Emitter::Polymorphic* ptr_light{ nullptr };
		std::uint32_t emitter_id{ UINT32_MAX }; // If used, but not set correctly, will throw a std::overflow_error
		BxDF::Polymorphic* ptr_material{ nullptr };

		Vertex() = default;

		Vertex(
			Ray::Intersection const& idata,
			Colour const& throughput,
			std::float_t const pdf_forward,
			std::float_t const pdf_reverse,
			bool const f_dirac,
			bool const f_emitter,
			bool const f_camera = false
		)
			: idata( idata )
			, throughput( throughput )
			, pdf_forward( pdf_forward )
			, pdf_reverse( pdf_reverse )
			, f_dirac( f_dirac )
			, f_emitter( f_emitter )
			, f_camera( f_camera ) // See note above
		{};

		// TODO dirac
		Double3 const& get_normal() const { return idata.orthogonal.normal(); };

		Double3 const& get_point() const { return idata.point; };

	};

};
