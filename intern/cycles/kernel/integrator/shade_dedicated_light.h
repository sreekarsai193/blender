/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2023 Blender Foundation */

#pragma once

#include "kernel/integrator/path_state.h"
#include "kernel/integrator/shade_surface.h"
#include "kernel/light/distant.h"
#include "kernel/light/light.h"

CCL_NAMESPACE_BEGIN

#ifdef __SHADOW_LINKING__

ccl_device_inline bool shadow_linking_light_sample_from_intersection(
    KernelGlobals kg,
    ccl_private const Intersection &ccl_restrict isect,
    ccl_private const Ray &ccl_restrict ray,
    ccl_private LightSample *ccl_restrict ls)
{
  const int lamp = isect.prim;

  const ccl_global KernelLight *klight = &kernel_data_fetch(lights, lamp);
  const LightType type = LightType(klight->type);

  if (type == LIGHT_DISTANT) {
    return distant_light_sample_from_intersection(kg, ray.D, lamp, ls);
  }

  return light_sample_from_intersection(kg, &isect, ray.P, ray.D, ls);
}

ccl_device_inline float shadow_linking_light_sample_mis_weight(KernelGlobals kg,
                                                               IntegratorState state,
                                                               const uint32_t path_flag,
                                                               const ccl_private LightSample *ls,
                                                               const float3 P)
{
  if (ls->type == LIGHT_DISTANT) {
    return light_sample_mis_weight_forward_distant(kg, state, path_flag, ls);
  }

  return light_sample_mis_weight_forward_lamp(kg, state, path_flag, ls, P);
}

/* Setup ray for the shadow path.
 * Expects that the current state of the ray is the one calculated by the surface bounce, and the
 * intersection corresponds to a point on an emitter. */
ccl_device void shadow_linking_setup_ray_from_intersection(
    IntegratorState state,
    ccl_private Ray *ccl_restrict ray,
    ccl_private const Intersection *ccl_restrict isect)
{
  kernel_assert(isect->type == PRIMITIVE_LAMP);

  /* The ray->tmin follows the value configured at the surface bounce.
   * it is the same for the continued main path and for this shadow ray. There is no need to push
   * it forward here. */

  ray->tmax = isect->t;

  /* Use the same self intersection primitives as the main path.
   * Those are copied to the dedicated storage from the main intersection after the surface bounce,
   * but before the main intersection is re-used to find light to trace a ray to. */
  ray->self.object = INTEGRATOR_STATE(state, shadow_link, last_isect_object);
  ray->self.prim = INTEGRATOR_STATE(state, shadow_link, last_isect_prim);

  // TODO: Support mesh lights.
  ray->self.light_object = OBJECT_NONE;
  ray->self.light_prim = PRIM_NONE;
  ray->self.light = isect->prim;
}

ccl_device void shadow_linking_shade(KernelGlobals kg, IntegratorState state)
{
  /* Read intersection from integrator state into local memory. */
  Intersection isect ccl_optional_struct_init;
  integrator_state_read_isect(state, &isect);

  /* Read ray from integrator state into local memory. */
  Ray ray ccl_optional_struct_init;
  integrator_state_read_ray(state, &ray);

  LightSample ls ccl_optional_struct_init;
  const bool use_light_sample = shadow_linking_light_sample_from_intersection(kg, isect, ray, &ls);
  if (!use_light_sample) {
    /* No light to be sampled, so no direct light contribution either. */
    return;
  }

  ShaderDataCausticsStorage emission_sd_storage;
  ccl_private ShaderData *emission_sd = AS_SHADER_DATA(&emission_sd_storage);
  const Spectrum light_eval = light_sample_shader_eval(kg, state, emission_sd, &ls, ray.time);
  if (is_zero(light_eval)) {
    return;
  }

  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
  if (!is_light_shader_visible_to_path(ls.shader, path_flag)) {
    return;
  }

  /* MIS weighting. */
  float mis_weight = 1.0f;
  if (!(path_flag & PATH_RAY_MIS_SKIP)) {
    mis_weight = shadow_linking_light_sample_mis_weight(kg, state, path_flag, &ls, ray.P);
  }

  const Spectrum bsdf_spectrum = light_eval * mis_weight *
                                 INTEGRATOR_STATE(state, shadow_link, dedicated_light_weight);

  shadow_linking_setup_ray_from_intersection(state, &ray, &isect);

  /* Branch off shadow kernel. */
  IntegratorShadowState shadow_state = integrate_direct_light_shadow_init_common(
      kg, state, &ls, &ray, bsdf_spectrum, 0);

  /* No need to update the volume stack as the surface bounce already performed enter-exit check.
   */

  const uint32_t shadow_flag = INTEGRATOR_STATE(state, path, flag);

  if (kernel_data.kernel_features & KERNEL_FEATURE_LIGHT_PASSES) {
    /* The diffuse and glossy pass weights are written into the main path as part of the path
     * configuration at a surface bounce. */
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, pass_diffuse_weight) = INTEGRATOR_STATE(
        state, path, pass_diffuse_weight);
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, pass_glossy_weight) = INTEGRATOR_STATE(
        state, path, pass_glossy_weight);
  }

  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, flag) = shadow_flag;

  // TODO: Disable path guiding for this shadow ray?
}

#endif /* __SHADOW_LINKING__ */

ccl_device void integrator_shade_dedicated_light(KernelGlobals kg,
                                                 IntegratorState state,
                                                 ccl_global float *ccl_restrict /*render_buffer*/)
{
  PROFILING_INIT(kg, PROFILING_SHADE_DEDICATED_LIGHT);

#ifdef __SHADOW_LINKING__
  shadow_linking_shade(kg, state);

  /* Restore self-intersection check primitives in the main state before returning to the
   * intersect_closest() state. */
  shadow_linking_restore_last_primitives(state);
#else
  kernel_assert(!"integrator_intersect_dedicated_light is not supposed to be scheduled");
#endif

  integrator_shade_surface_next_kernel<DEVICE_KERNEL_INTEGRATOR_SHADE_DEDICATED_LIGHT>(kg, state);
}

CCL_NAMESPACE_END
