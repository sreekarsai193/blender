/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2023 Blender Foundation */

#pragma once

#include "kernel/bvh/bvh.h"
#include "kernel/integrator/path_state.h"
#include "kernel/integrator/shade_surface.h"
#include "kernel/integrator/shadow_linking.h"
#include "kernel/light/light.h"

CCL_NAMESPACE_BEGIN

#ifdef __SHADOW_LINKING__

/* Pick a light for tracing a shadow ray for the shadow linking.
 * Picks a random light which is intersected by the given ray, and stores the intersection result.
 * If no lights were hit false is returned. */
ccl_device bool shadow_linking_pick_light_intersection(KernelGlobals kg,
                                                       IntegratorState state,
                                                       ccl_private const Ray *ccl_restrict ray,
                                                       ccl_private Intersection *ccl_restrict
                                                           isect)
{
  uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);

  const int last_prim = INTEGRATOR_STATE(state, isect, prim);
  const int last_object = INTEGRATOR_STATE(state, isect, object);
  const int last_type = INTEGRATOR_STATE(state, isect, type);

  /* The lights_intersect() has a "refining" behavior: it chooses intersection closer to the
   * current intersection's distance. Hence initialize the fields which are accessed prior to
   * recording an intersection. */
  isect->t = FLT_MAX;
  isect->prim = PRIM_NONE;

  // TODO: Support mesh emitters.

  // TODO: Support multiple light sources.

  // TODO: Distant lights.

  // TODO: Only if ray is not fully occluded.

  // TODO: What of the actual shadow ray hits the same light through a semi-transparent surface?

  const int receiver_forward = light_link_receiver_forward(kg, state);

  if (!lights_intersect_shadow_linked(
          kg, ray, isect, last_prim, last_object, last_type, path_flag, receiver_forward))
  {
    return false;
  }

  return true;
}

/* Check whether a special shadow ray is needed to calculate direct light contribution which comes
 * from emitters which are behind objects which are blocking light for the main path, but are
 * excluded from blocking light via shadow linking.
 *
 * If a special ray is needed a blocked light kernel is scheduled and true is returned, otherwise
 * false is returned. */
ccl_device bool shadow_linking_intersect(KernelGlobals kg, IntegratorState state)
{
  /* Verify that the kernel is only scheduled if it is actually needed. */
  kernel_assert(shadow_linking_scene_need_shadow_ray(kg, state));

  /* Read ray from integrator state into local memory. */
  Ray ray ccl_optional_struct_init;
  integrator_state_read_ray(state, &ray);

  Intersection isect ccl_optional_struct_init;
  if (!shadow_linking_pick_light_intersection(kg, state, &ray, &isect)) {
    /* No light is hit, no need in the extra shadow ray for the direct light. */
    return false;
  }

  /* Make a copy of primitives needed by the main path self-intersection check before writing the
   * new intersection. Those primitives will be restored before the main path is returned to the
   * intersect_closest state. */
  shadow_linking_store_last_primitives(state);

  /* Write intersection result into global integrator state memory, so that the
   * shade_dedicated_light kernel can use it for calculation of the light sample, */
  integrator_state_write_isect(state, &isect);

  integrator_path_next(kg,
                       state,
                       DEVICE_KERNEL_INTEGRATOR_INTERSECT_DEDICATED_LIGHT,
                       DEVICE_KERNEL_INTEGRATOR_SHADE_DEDICATED_LIGHT);

  return true;
}

#endif /* __SHADOW_LINKING__ */

ccl_device void integrator_intersect_dedicated_light(KernelGlobals kg, IntegratorState state)
{
  PROFILING_INIT(kg, PROFILING_INTERSECT_DEDICATED_LIGHT);

#ifdef __SHADOW_LINKING__
  if (shadow_linking_intersect(kg, state)) {
    return;
  }
#else
  kernel_assert(!"integrator_intersect_dedicated_light is not supposed to be scheduled");
#endif

  integrator_shade_surface_next_kernel<DEVICE_KERNEL_INTEGRATOR_INTERSECT_DEDICATED_LIGHT>(kg,
                                                                                           state);
}

CCL_NAMESPACE_END
