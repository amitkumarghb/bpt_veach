// Copyright (c) 2025 Thomas Klietsch, all rights reserved.
//
// Licensed under the GNU Lesser General Public License, version 3.0 or later
//
// This program is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or ( at your option ) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General
// Public License along with this program.If not, see < https://www.gnu.org/licenses/>. 

#pragma once

#include <cstdint>
#include <memory>
#include <tuple>
#include <vector>

#include "../bxdf/polymorphic.hpp"
#include "../bxdf/shading_correction.hpp"
#include "../colour/colour.hpp"
#include "../epsilon.hpp"
#include "../integrator/vertex.hpp"
#include "../mathematics/double3.hpp"
#include "../random/mersenne.hpp"
#include "../random/mersenne.hpp"
#include "../ray/section.hpp"
#include "../render/camera.hpp"
#include "../render/config.hpp"
#include "../render/scene.hpp"
#include "../render/sensor.hpp"

namespace Integrator
{

	// Veach thesis
	// Bidrectional path tracer
	class BDPT
	{

	private:

		std::uint8_t const max_path_length{ 3 };
		std::uint16_t const max_samples{ 1 };

		Render::Camera const camera;
		Render::Scene const scene;
		Render::Sensor& sensor;

		Random::Mersenne prng;

		// Veach 273
		inline std::double_t MIS( std::double_t value ) const
		{
			// Balance
			return value;
			// Power, beta=2
			//return value * value;
		};

	public:

		BDPT() = delete;

		BDPT(
			Render::Camera const& camera,
			Render::Sensor& sensor,
			Render::Scene const& scene,
			Render::Config const& config
		)
			: camera( camera )
			, sensor( sensor )
			, scene( scene )
			, max_path_length( config.max_path_length )
			, max_samples( config.max_samples )
		{};

		void process(
			std::uint16_t const x,
			std::uint16_t const y
		)
		{
			prng = Random::Mersenne( ( ( x + 1 ) * 0x1337 ) + ( ( y + 1 ) * 0xbeef ) );

			Colour accumulate( Colour::Black );

			for ( std::uint16_t sample{ 0 }; sample < max_samples; ++sample )
			{
				// Generate paths
				auto const emission_path = trace_emission_path();
				auto const camera_path = trace_camera_path( x, y );
				// Check if paths hit an element type sampled from the other path
				bool const f_hit_camera = emission_path.back().f_camera; // Only possible for cameras with an area lens
				bool const f_hit_emitter = camera_path.back().f_emitter;
				// Subtract one (1) from path if hit special case above, full path is only evaluated in Type 1)
				std::uint8_t const n_emission_path = emission_path.size() - ( f_hit_camera ? 1 : 0 );
				std::uint8_t const n_camera_path = camera_path.size() - ( f_hit_emitter ? 1 : 0 );

				// Three (3) types of connections

				// Type 1) Direct hit on an emitter
				if ( ( n_camera_path > 0 ) && f_hit_emitter )
				{
					// s=0, t>1
					// Fully traced camera path, hitting an area/environment emitter
					// No path visibility check is needed
					std::uint8_t const t = n_camera_path + 1;
					Integrator::Vertex const& vertex = camera_path[t - 1];
					if ( !vertex.f_dirac )
					{
						Integrator::Vertex const& previous_vertex = camera_path[t - 2];
						Double3 const evaluate_direction = ( previous_vertex.get_point() - vertex.get_point() ).normalise();
						Double3 const evaluate_point = vertex.get_point();
						accumulate +=
							vertex.throughput
							* vertex.ptr_light->radiance(evaluate_point, evaluate_direction)
							* Weight( 0, t, emission_path, camera_path );
					}
				}

				// Type 1) Direct hit on an emitter or a camera lens
				if ( ( n_emission_path > 0 ) && f_hit_camera )
				{
					// Needs a camera with a lens radius large than zero
					// s>1, t=0
					// Fully traced emission path, hitting a camera lens
					// Path visibility is therefore true
				}

				// Type 2) Connecting camera path to an emitter
				if ( n_camera_path > 0 )
				{
					// Evaluate the camera path, next event estimator (NEE)
					// unless it is a camera (t=0) or emitter (t=end)

					Integrator::Vertex const& vertex_emitter = emission_path[0];
					std::double_t const emitter_select_prb = scene.emitter_select_probability( vertex_emitter.emitter_id );
					Double3 const& emitter_point = vertex_emitter.get_point();
					for ( std::uint8_t t{ 1 };t < n_camera_path;++t )
					{
						Integrator::Vertex const& vertex = camera_path[t];
						if ( vertex.f_dirac )
							continue;
						Double3 const& surface_point = vertex.get_point();
						Double3 const delta = emitter_point - surface_point;
						Double3 const evaluate_direction = delta.normalise();
						std::double_t const evaluate_distance = delta.magnitude();
						Ray::Section const ray( surface_point, evaluate_direction, EPSILON_RAY );
						if ( !scene.occluded( ray, evaluate_distance - 2. * EPSILON_RAY ) )
						{
							Double3 const previous_direction = ( camera_path[t - 1].get_point() - surface_point ).normalise();
							accumulate +=
								vertex.throughput
								* vertex_emitter.ptr_light->radiance(emitter_point, -evaluate_direction)
								* vertex.ptr_material->factor(evaluate_direction, previous_direction, vertex.idata, BxDF::TraceMode::Radiance)
								* Gprime(vertex, vertex_emitter)
								* Weight(1, t, emission_path, camera_path)
								/ ( vertex_emitter.ptr_light->pdf_A( emitter_point, -evaluate_direction ) * emitter_select_prb );
						}
					}
				}

				// Type 2) Connecting emitter path to a camera lens
				if ( n_emission_path > 0 )
				{
					// Evaluate the emmision path, particle/light trace
					// unless it is a emitter (s=0) or camera (s=end)

					Integrator::Vertex const& vertex_camera = camera_path[0];
					Double3 const lens_point = camera.sample_lens( prng );
					for ( std::uint8_t s{ 1 };s < n_emission_path;++s )
					{
						Integrator::Vertex const& vertex = emission_path[s];
						if ( vertex.f_dirac )
							continue;
						auto const [x, y, f_valid] = camera.sensor( vertex.get_point(), lens_point );
						if ( f_valid )
						{
							Double3 const delta = vertex.get_point() - lens_point;
							Double3 const evaluate_direction = delta.normalise();
							std::double_t const evaluate_distance = delta.magnitude();
							Ray::Section const ray( lens_point, evaluate_direction, EPSILON_RAY );
							if ( !scene.occluded( ray, evaluate_distance - 2. * EPSILON_RAY ) )
							{
								Double3 const previous_direction = ( emission_path[s - 1].get_point() - vertex.get_point() ).normalise();
								// Note: the result is stored in a different buffer than camera traces (pixel)
								sensor.splash( x, y,
									vertex.throughput * ShadingCorrection( evaluate_direction, vertex.idata.from_direction, vertex.idata, BxDF::TraceMode::Importance )
									* vertex.ptr_material->factor( -evaluate_direction, previous_direction, vertex.idata, BxDF::TraceMode::Importance )
									* Gprime( vertex, vertex_camera )
									* Weight( s, 1, emission_path, camera_path )
									/ camera.We( lens_point, evaluate_direction )
								);
							}
						}
					}
				}

				// Type 3) Connect all (non dirac) material vertices from one path to the other

				// No possible connections
				if ( n_emission_path < 2 && n_camera_path < 2 )
					continue;

				for ( std::uint8_t s{ 2 };s <= n_emission_path;++s )
				{
					Integrator::Vertex const& s_vertex = emission_path[s - 1];
					if ( s_vertex.f_dirac )
						continue;
					for ( std::uint8_t t{ 2 };t <= n_camera_path;++t )
					{
						Integrator::Vertex const& t_vertex = camera_path[t - 1];
						if ( t_vertex.f_dirac )
							continue;

						// Limit to k = s + t - 1. Faster render, but can appear darker
						//if ( s + t > max_path_length )
						//	continue;

						// Connecting edge, Veach 301
						Double3 const delta = t_vertex.get_point() - s_vertex.get_point();
						Double3 const evaluate_direction = delta.normalise();
						std::double_t const evaluate_distance = delta.magnitude();

						// The visibility term in G, is evaluated independently
						bool const f_occluded = scene.occluded( Ray::Section( s_vertex.get_point(), evaluate_direction, EPSILON_RAY ), evaluate_distance - 2. * EPSILON_RAY );
						if ( !f_occluded )
						{
							Double3 const previous_direction_emission = ( emission_path[s - 2].get_point() - s_vertex.get_point() ).normalise();
							Double3 const previous_direction_camera = ( camera_path[t - 2].get_point() - t_vertex.get_point() ).normalise();

							accumulate +=
								// Flow from emitter
								s_vertex.throughput * ShadingCorrection( evaluate_direction, s_vertex.idata.from_direction, s_vertex.idata, BxDF::TraceMode::Importance )
								* s_vertex.ptr_material->factor( evaluate_direction, previous_direction_emission, s_vertex.idata, BxDF::TraceMode::Importance )
								// Flow from camera
								* t_vertex.throughput
								* t_vertex.ptr_material->factor( -evaluate_direction, previous_direction_camera, t_vertex.idata, BxDF::TraceMode::Radiance )
								// G and MIS weight
								* Gprime( s_vertex, t_vertex )
								* Weight( s, t, emission_path, camera_path );
						}
					} // end t
				} // end s

			} // end sample loop

			sensor.pixel( x, y, accumulate );
		}; // end process

	private:

		std::vector<Integrator::Vertex> trace_emission_path()
		{
			// Veach 92
			// Particle/Importance tracing.
			// From emitter (wi), BxDF samples wo
			std::vector<Integrator::Vertex> vertices;
			std::uint32_t const emitter_id = scene.random_emitter( prng );
			auto const [p_emitter, emitter_select_probability]
				= scene.emitter( emitter_id );

			auto const [emitter_factor, emitter_point, emitter_direction, emitter_normal, emitter_pdf_W, emitter_pdf_A, emitter_cos_theta]
				= p_emitter->emit( prng );

			Colour throughput = emitter_factor * emitter_cos_theta / ( emitter_select_probability * emitter_pdf_W * emitter_pdf_A );

			// Light vertex is y0
			Ray::Intersection idata;
			idata.point = emitter_point;
			if ( !p_emitter->is_dirac() )
				idata.orthogonal = Orthogonal( emitter_normal );
			std::float_t pdf_reverse = emitter_select_probability * emitter_pdf_A;
			std::float_t pdf_forward = p_emitter->is_dirac()
				? emitter_pdf_W
				: emitter_pdf_W / emitter_cos_theta;
			vertices.emplace_back( Integrator::Vertex( idata, throughput, pdf_forward, pdf_reverse, p_emitter->is_dirac(), true ) );
			vertices[0].ptr_light = p_emitter.get();
			vertices[0].emitter_id = emitter_id;

			Ray::Section ray( emitter_point, emitter_direction, EPSILON_RAY );
			std::uint8_t depth{ 1 };

			while ( 1 )
			{
				auto [f_hit, hit_distance, idata] = scene.intersect( ray );
				if ( !f_hit )
					return vertices;

				std::shared_ptr<BxDF::Polymorphic> const& p_material = scene.material( idata.material_id );
				auto [bxdf_colour, bxdf_direction, bxdf_event, bxdf_pdf_W, bxdf_cos_theta]
					= p_material->sample( idata, BxDF::TraceMode::Importance, prng );

				pdf_forward = bxdf_pdf_W / bxdf_cos_theta;

				switch ( bxdf_event )
				{
					default:
					case BxDF::Event::None:
					case BxDF::Event::Emission:
					{
						return vertices;
					}
					case BxDF::Event::Diffuse:
					{
						if ( depth == 1 && p_emitter->is_dirac() )
							// Impossible to intersect
							pdf_reverse = 0.f;
						else
						{
							auto const [evaluate_colour, evaluate_pdf_W, evaluate_cos_theta]
								= p_material->evaluate( -ray.direction, bxdf_direction, idata, BxDF::TraceMode::Importance );
							pdf_reverse = evaluate_pdf_W / evaluate_cos_theta;
						}
						Integrator::Vertex vertex = Integrator::Vertex( idata, throughput, pdf_forward, pdf_reverse, false, false );
						vertex.ptr_material = p_material.get();
						vertex.G = Gprime( vertex, vertices.back() );
						vertices.emplace_back( vertex );
						throughput *= ( bxdf_colour / pdf_forward ) * ShadingCorrection( bxdf_direction, idata.from_direction, idata, BxDF::TraceMode::Importance );
						break;
					}
					case BxDF::Event::Reflect:
					{
						pdf_reverse = ( depth == 1 & p_emitter->is_dirac() )
							? 0.f
							: pdf_forward;
						Integrator::Vertex vertex = Integrator::Vertex( idata, throughput, pdf_forward, pdf_reverse, true, false );
						vertex.ptr_material = p_material.get();
						vertex.G = Gprime( vertex, vertices.back() );
						vertices.emplace_back( vertex );
						throughput *= bxdf_colour * ShadingCorrection( bxdf_direction, idata.from_direction, idata, BxDF::TraceMode::Importance );
						break;
					}
				} // end switch

				if ( ++depth > max_path_length )
					break;

				ray = Ray::Section( idata.point, bxdf_direction, EPSILON_RAY );
			} // end trace loop

			return vertices;
		};

		std::vector<Integrator::Vertex> trace_camera_path(
			std::uint16_t const x,
			std::uint16_t const y
		)
		{
			// Veach 92
			// Path/Radiance tracing.
			// From camera (wo), BxDF samples wi
			std::vector<Integrator::Vertex> vertices;
			Ray::Section ray = camera.generate_ray( x, y, prng );

			auto [pdf_W, pdf_A, cos_theta]
				= camera.evaluate( ray.origin, ray.direction );

			std::float_t pdf_forward = pdf_W / cos_theta;
			std::float_t pdf_reverse = pdf_A;

			// Camera vertex is z0
			Ray::Intersection idata;
			idata.point = ray.origin;
			idata.orthogonal = Orthogonal( camera.lens_normal( ray.origin ) );
			vertices.emplace_back( Integrator::Vertex( idata, Colour::White, pdf_forward, pdf_reverse, camera.is_dirac(), false ) );

			std::uint8_t depth{ 1 };

			Colour throughput{ Colour::White * camera.We( ray.origin, ray.direction ) / pdf_forward };

			// Trace loop
			while ( 1 )
			{
				auto const [f_hit, hit_distance, idata] = scene.intersect( ray );
				if ( !f_hit )
					return vertices;

				std::shared_ptr<BxDF::Polymorphic> const& p_material = scene.material( idata.material_id );
				auto const [bxdf_colour, bxdf_direction, bxdf_event, bxdf_pdf_W, bxdf_cos_theta]
					= p_material->sample( idata, BxDF::TraceMode::Radiance, prng );

				pdf_forward = bxdf_pdf_W / bxdf_cos_theta;

				switch ( bxdf_event )
				{
					default:
					case BxDF::Event::None:
					{
						return vertices;
					}
					case BxDF::Event::Emission:
					{
						auto const [p_light, select_prb] = scene.emitter( p_material->emitter_id() );
						Integrator::Vertex vertex = Integrator::Vertex( idata, throughput, 1, 1, false, true );
						vertex.ptr_material = p_material.get(); // Just for access to emitter_id in MIS // TODO
						vertex.ptr_light = p_light.get();
						vertex.G = Gprime( vertex, vertices.back() );
						vertices.emplace_back( vertex );
						return vertices;
					}
					case BxDF::Event::Diffuse:
					{
						if ( depth == 1 && camera.is_dirac() )
							pdf_reverse = 0.f;
						else
						{
							auto const [evaluate_colour, evaluate_pdf_W, evaluate_cos_theta]
								= p_material->evaluate( -ray.direction, bxdf_direction, idata, BxDF::TraceMode::Radiance );
							pdf_reverse = evaluate_pdf_W / evaluate_cos_theta;
						}
						Integrator::Vertex vertex = Integrator::Vertex( idata, throughput, pdf_forward, pdf_reverse, false, false );
						vertex.ptr_material = p_material.get();
						vertex.G = Gprime( vertex, vertices.back() );
						vertices.emplace_back( vertex );
						throughput *= bxdf_colour / pdf_forward;
						break;
					}
					case BxDF::Event::Reflect:
					{
						pdf_reverse = ( depth == 1 & camera.is_dirac() )
							? 0.f
							: pdf_forward;
						Integrator::Vertex vertex = Integrator::Vertex( idata, throughput, pdf_forward, pdf_reverse, true, false );
						vertex.ptr_material = p_material.get();
						vertex.G = Gprime( vertex, vertices.back() );
						vertices.emplace_back( vertex );
						throughput *= bxdf_colour;
						break;
					}
				} // end switch

				if ( ++depth > max_path_length )
					break;

				ray = Ray::Section( idata.point, bxdf_direction, EPSILON_RAY );
			} // end trace loop

			return vertices;
		};

		// Veach // TODO
		std::double_t Gprime(
			Integrator::Vertex const& vertex_a,
			Integrator::Vertex const& vertex_b
		) const
		{
			// Assumes that vertices a and b are not dirac
			Double3 const delta = vertex_b.get_point() - vertex_a.get_point();
			Double3 const evaluate_direction = delta.normalise();
			return
				std::max( 0., evaluate_direction.dot( vertex_a.get_normal() ) )
				* std::max( 0., -( evaluate_direction.dot( vertex_b.get_normal() ) ) )
				/ ( delta.dot( delta ) );
		};

		struct Node
		{
			std::double_t p_forward{ 0. }; // Flow from emitter
			std::double_t p_reverse{ 0. }; // Flow from camera
			bool f_dirac{ false };
		};

		std::double_t Weight(
			std::uint8_t const s,
			std::uint8_t const t,
			std::vector<Integrator::Vertex> const& emission_path,
			std::vector<Integrator::Vertex> const& camera_path
		) const
		{
			std::uint8_t const k = s + t - 1;

			Integrator::Vertex const& s_vertex = emission_path[s - 1];
			Integrator::Vertex const& t_vertex = camera_path[t - 1];

			// s/emission vertex
			std::double_t pdf_s_forward{ 0. };
			std::double_t pdf_s_reverse{ 0. };
			// t/camera vertex
			std::double_t pdf_t_forward{ 0. };
			std::double_t pdf_t_reverse{ 0. };

			// Evaluate connection vertices
			if ( s == 0 )
			{
				if ( t_vertex.f_emitter )
				{
					Double3 const evaluate_direction = ( camera_path[t - 2].get_point() - t_vertex.get_point() ).normalise();
					auto const [emitter_pdf_W, emitter_pdf_A, emitter_cos_theta]
						= t_vertex.ptr_light->pdf_Le( t_vertex.get_point(), evaluate_direction );
					pdf_t_forward = emitter_pdf_A * scene.emitter_select_probability( t_vertex.ptr_material->emitter_id() );
					pdf_t_reverse = emitter_pdf_W / emitter_cos_theta;
				}
			}
			else if ( t == 0 )
			{
				Double3 const& point = s_vertex.get_point();
				Double3 const evaluate_direction = ( emission_path[s - 2].get_point() - point ).normalise();
				auto const [pdf_W, pdf_A, cos_theta]
					= camera.evaluate( point, evaluate_direction );
				pdf_s_forward = pdf_A;
				pdf_s_reverse = pdf_W / cos_theta;
			}
			else
			{
				Double3 const& s_vertex_point = s_vertex.get_point();
				Double3 const& t_vertex_point = t_vertex.get_point();

				{
					Double3 const evaluate_direction = ( t_vertex_point - s_vertex_point ).normalise();
					if ( s == 1 )
					{
						Double3 const& vertex_normal = s_vertex.get_normal();
						std::float_t const pdfW = s_vertex.ptr_light->pdf_W( s_vertex_point, evaluate_direction );
						pdf_s_forward = s_vertex.ptr_light->is_dirac()
							? pdfW
							: pdfW / vertex_normal.dot( evaluate_direction );
						pdf_s_reverse = s_vertex.pdf_reverse;
					}
					else
					{
						Double3 const& vertex_normal = s_vertex.get_normal();
						Double3 const previous_direction = ( emission_path[s - 2].get_point() - s_vertex_point ).normalise();
						pdf_s_forward = s_vertex.ptr_material->pdf( evaluate_direction, previous_direction, s_vertex.idata ) / vertex_normal.dot( evaluate_direction );
						pdf_s_reverse = s_vertex.ptr_material->pdf( previous_direction, evaluate_direction, s_vertex.idata ) / vertex_normal.dot( previous_direction );
					}
				}

				{
					Double3 const evaluate_direction = ( s_vertex_point - t_vertex_point ).normalise();
					if ( t == 1 )
					{
						// Dirac camera
						Double3 const& vertex_normal = t_vertex.get_normal();
						auto const [pdf_W, pdf_A, cos_theta]
							= camera.evaluate( t_vertex_point, evaluate_direction );
						pdf_t_forward = pdf_W / vertex_normal.dot( evaluate_direction );
						pdf_t_reverse = t_vertex.pdf_reverse;
					}
					else
					{
						Double3 const& vertex_normal = t_vertex.get_normal();
						Double3 const previous_direction = ( camera_path[t - 2].get_point() - t_vertex_point ).normalise();
						pdf_t_forward = t_vertex.ptr_material->pdf( evaluate_direction, previous_direction, t_vertex.idata ) / vertex_normal.dot( evaluate_direction );
						pdf_t_reverse = t_vertex.ptr_material->pdf( previous_direction, evaluate_direction, t_vertex.idata ) / vertex_normal.dot( previous_direction );
					}
				}
			}

			// Catch NaN and negative pdf
			if ( std::isnan( pdf_s_forward ) || ( pdf_s_forward < 0. ) )
				pdf_s_forward = 0.;
			if ( std::isnan( pdf_s_reverse ) || ( pdf_s_reverse < 0. ) )
				pdf_s_reverse = 0.;
			if ( std::isnan( pdf_t_forward ) || ( pdf_t_forward < 0. ) )
				pdf_t_forward = 0.;
			if ( std::isnan( pdf_t_reverse ) || ( pdf_t_reverse < 0. ) )
				pdf_t_reverse = 0.;

			std::vector<Node> node;
			node.reserve( k + 2 );

			// Veach 306
			// x_bar = x0 ... xk,
			// starting from emitter to camera

			// ps+1 ... pk+1, starting at ps (light sub path)
			//
			// p0     PA(x0)
			// --   = ------------------
			// p1     P(x1 -> x0) G(x0,x1)
			//
			// pi+1   P(xi-1 -> xi) G(x-1,xi)
			// ---- = -----------------------  , 0<i<k
			// pi     P(xi+1 -> xi) G(x+1,xi)
			//
			// pk+1   P(xk-1 -> xk) G(xk-1,xk)
			// ---- = ------------------
			// pk     PA(xk)
			//
			// ps-1 ... p0, starting at ps (eye/camera sub path)
			// uses reciprocal of above equations

			// Fill in nodes
			for ( std::uint8_t i{ 0 }; i < s - 1; ++i )
			{
				node[i].p_forward = emission_path[i].pdf_forward * emission_path[i + 1].G;
				node[i].p_reverse = i == 0
					? emission_path[0].pdf_reverse
					: emission_path[i].pdf_reverse * emission_path[i].G;
				node[i].f_dirac = emission_path[i].f_dirac;
			}
			if ( s > 0 )
			{
				node[s - 1].p_forward = ( ( s - 1 ) == k )
					? pdf_s_forward
					: pdf_s_forward * Gprime( s_vertex, t_vertex );
				node[s - 1].p_reverse = s == 1
					? pdf_s_reverse
					: pdf_s_reverse * emission_path[s - 1].G;
				node[s - 1].f_dirac = emission_path[s - 1].f_dirac;
			}

			for ( std::uint8_t i{ 0 }; i < t - 1; ++i )
			{
				node[k - i].p_forward = i == 0
					? camera_path[0].pdf_reverse
					: camera_path[i].pdf_reverse * camera_path[i].G;
				node[k - i].p_reverse = camera_path[i].pdf_forward * camera_path[i + 1].G;
				node[k - i].f_dirac = camera_path[i].f_dirac;
			}
			if ( t > 0 )
			{
				node[k - ( t - 1 )].p_forward = t == 1
					? pdf_t_reverse
					: pdf_t_reverse * camera_path[t - 1].G;
				node[k - ( t - 1 )].p_reverse = ( ( t - 1 ) == k )
					? pdf_t_forward
					: pdf_t_forward * Gprime( s_vertex, t_vertex );
				node[k - ( t - 1 )].f_dirac = camera_path[t - 1].f_dirac;
			}

			// Calculate all (relative) path weights
			std::double_t sum_path{ 1.0 }; // Self weight is one (1)

			std::double_t p_k{ 1.0 };
			for ( std::uint8_t i{ s }; i <= k; ++i )
			{
				if ( i == 0 )
				{
					p_k *= node[0].p_reverse / node[1].p_reverse;
					if ( node[1].f_dirac )
						continue;
				}
				else if ( i == k )
				{
					if ( camera.is_dirac() )
						break;
					p_k *= node[k - 1].p_forward / node[k].p_forward;
				}
				else
				{
					p_k *= node[i - 1].p_forward / node[i + 1].p_reverse;
					if ( node[i].f_dirac || node[i + 1].f_dirac )
						continue;
				}
				sum_path += MIS( p_k );
			}

			p_k = 1.0;
			for ( std::uint8_t i{ s }; i > 0; --i )
			{
				if ( i == ( k + 1 ) )
				{
					p_k *= node[k].p_forward / node[k - 1].p_forward;
					if ( node[k - 1].f_dirac )
						continue;
				}
				else if ( i == 1 )
				{
					if ( emission_path[0].ptr_light->is_dirac() )
						break;
					p_k *= node[1].p_reverse / node[0].p_reverse;
				}
				else
				{
					p_k *= node[i].p_reverse / node[i - 2].p_forward;
					if ( node[i - 1].f_dirac || node[i - 2].f_dirac )
						continue;
				}
				sum_path += MIS( p_k );
			}

			// Return pdf of path weights
			return 1. / ( sum_path );
		};

	}; // end bdpt class

};
