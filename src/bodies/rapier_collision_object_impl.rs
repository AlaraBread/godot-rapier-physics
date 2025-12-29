use rapier::geometry::ColliderHandle;

use super::rapier_area::MonitorInfo;
use super::rapier_collision_object::IRapierCollisionObject;
use super::rapier_collision_object_base::CollisionObjectShape;
use super::rapier_collision_object_base::RapierCollisionObjectBase;
use crate::rapier_wrapper::prelude::PhysicsEngine;
use crate::servers::rapier_physics_singleton::PhysicsIds;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::servers::rapier_physics_singleton::PhysicsSpaces;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::servers::rapier_physics_singleton::get_id_rid;
use crate::shapes::rapier_shape::IRapierShape;
use crate::types::Transform;
impl RapierCollisionObjectBase {
    pub(super) fn recreate_shapes(
        collision_object: &mut dyn IRapierCollisionObject,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        use hashbrown::HashMap;
        let mut handle_mapping: HashMap<ColliderHandle, ColliderHandle> = HashMap::new();
        for i in 0..collision_object.get_base().get_shape_count() as usize {
            if collision_object.get_base().state.shapes[i].disabled {
                continue;
            }
            let old_handle = collision_object.get_base().state.shapes[i].collider_handle;
            if old_handle != ColliderHandle::invalid() {
                collision_object.get_mut_base().state.shapes[i].collider_handle =
                    collision_object.get_base().destroy_shape(
                        collision_object.get_base().state.shapes[i],
                        i,
                        physics_spaces,
                        physics_engine,
                        physics_ids,
                    );
            }
            let new_handle = collision_object.create_shape(
                collision_object.get_base().state.shapes[i],
                i,
                physics_engine,
            );
            if old_handle != ColliderHandle::invalid() {
                handle_mapping.insert(old_handle, new_handle);
            }
            collision_object.get_mut_base().state.shapes[i].collider_handle = new_handle;
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[i],
                physics_engine,
            );
        }
        // Migrate monitored_objects for areas when handles change
        if let Some(area) = collision_object.get_mut_area() {
            let mut monitors_to_update: Vec<(
                (ColliderHandle, ColliderHandle),
                (ColliderHandle, ColliderHandle),
                MonitorInfo,
            )> = Vec::new();
            for ((other_handle, this_handle), monitor_info) in &area.state.monitored_objects {
                if let Some(&new_this_handle) = handle_mapping.get(this_handle) {
                    // This monitor's this_collider_handle was recreated, need to migrate it
                    monitors_to_update.push((
                        (*other_handle, *this_handle),
                        (*other_handle, new_this_handle),
                        *monitor_info,
                    ));
                }
            }
            // Remove old entries and insert with new handles
            for (old_key, new_key, monitor_info) in monitors_to_update {
                area.state.monitored_objects.remove(&old_key);
                area.state.monitored_objects.insert(new_key, monitor_info);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub(super) fn add_shape(
        collision_object: &mut dyn IRapierCollisionObject,
        p_shape_id: RapierId,
        p_transform: Transform,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    ) {
        let mut shape = CollisionObjectShape {
            xform: p_transform,
            id: p_shape_id,
            disabled: p_disabled,
            one_way_collision: false,
            one_way_collision_margin: 0.0,
            collider_handle: ColliderHandle::invalid(),
        };
        if !shape.disabled {
            shape.collider_handle = collision_object.create_shape(
                shape,
                collision_object.get_base().state.shapes.len(),
                physics_engine,
            );
            collision_object
                .get_base()
                .update_shape_transform(&shape, physics_engine);
        }
        collision_object.get_mut_base().state.shapes.push(shape);
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(p_shape_id, physics_ids)) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_id());
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }

    pub(super) fn shape_changed(
        collision_object: &mut dyn IRapierCollisionObject,
        shape_id: RapierId,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        use hashbrown::HashMap;
        let mut handle_mapping: HashMap<ColliderHandle, ColliderHandle> = HashMap::new();
        for i in 0..collision_object.get_base().state.shapes.len() {
            let shape = collision_object.get_base().state.shapes[i];
            if shape.id != shape_id || shape.disabled {
                continue;
            }
            let old_handle = collision_object.get_base().state.shapes[i].collider_handle;
            if old_handle != ColliderHandle::invalid() {
                collision_object.get_mut_base().state.shapes[i].collider_handle = collision_object
                    .get_base()
                    .destroy_shape(shape, i, physics_spaces, physics_engine, physics_ids);
            }
            let new_handle = collision_object.create_shape(
                collision_object.get_base().state.shapes[i],
                i,
                physics_engine,
            );
            if old_handle != ColliderHandle::invalid() {
                handle_mapping.insert(old_handle, new_handle);
            }
            collision_object.get_mut_base().state.shapes[i].collider_handle = new_handle;
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[i],
                physics_engine,
            );
        }
        // Migrate monitored_objects for areas when handles change
        if let Some(area) = collision_object.get_mut_area() {
            // Store the handle mapping for later use in process_new_event
            area.state
                .handle_migration_map
                .extend(handle_mapping.iter().map(|(k, v)| (*k, *v)));
            let mut monitors_to_update: Vec<(
                (ColliderHandle, ColliderHandle),
                (ColliderHandle, ColliderHandle),
                MonitorInfo,
            )> = Vec::new();
            for ((other_handle, this_handle), monitor_info) in &area.state.monitored_objects {
                if let Some(&new_this_handle) = handle_mapping.get(this_handle) {
                    // This monitor's this_collider_handle was recreated, need to migrate it
                    monitors_to_update.push((
                        (*other_handle, *this_handle),
                        (*other_handle, new_this_handle),
                        *monitor_info,
                    ));
                }
            }
            // Remove old entries and insert with new handles
            for (old_key, new_key, monitor_info) in monitors_to_update {
                area.state.monitored_objects.remove(&old_key);
                area.state.monitored_objects.insert(new_key, monitor_info);
            }
        }
        collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
    }

    pub(super) fn remove_shape_idx(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    ) {
        if p_index >= collision_object.get_base().state.shapes.len() {
            return;
        }
        let shape = &collision_object.get_base().state.shapes[p_index];
        if !shape.disabled {
            collision_object.get_base().destroy_shape(
                *shape,
                p_index,
                physics_spaces,
                physics_engine,
                physics_ids,
            );
        }
        let shape = collision_object.get_mut_base().state.shapes[p_index];
        collision_object.get_mut_base().state.shapes[p_index].collider_handle =
            ColliderHandle::invalid();
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(shape.id, physics_ids)) {
            shape
                .get_mut_base()
                .remove_owner(collision_object.get_base().get_id());
        }
        collision_object.get_mut_base().state.shapes.remove(p_index);
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
        collision_object
            .get_mut_base()
            .update_shapes_indexes(physics_engine);
    }

    pub(super) fn set_shape(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_shape: RapierId,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_shapes: &mut PhysicsShapes,
        physics_ids: &PhysicsIds,
    ) {
        if p_index >= collision_object.get_base().state.shapes.len() {
            return;
        }
        let old_handle = collision_object.get_base().state.shapes[p_index].collider_handle;
        collision_object.get_mut_base().state.shapes[p_index].collider_handle =
            collision_object.get_base().destroy_shape(
                collision_object.get_base().state.shapes[p_index],
                p_index,
                physics_spaces,
                physics_engine,
                physics_ids,
            );
        let shape = collision_object.get_base().state.shapes[p_index];
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(shape.id, physics_ids)) {
            shape
                .get_mut_base()
                .remove_owner(collision_object.get_base().get_id());
        }
        collision_object.get_mut_base().state.shapes[p_index].id = p_shape;
        if let Some(shape) = physics_shapes.get_mut(&get_id_rid(shape.id, physics_ids)) {
            shape
                .get_mut_base()
                .add_owner(collision_object.get_base().get_id());
        }
        let mut handle_mapping: hashbrown::HashMap<ColliderHandle, ColliderHandle> =
            hashbrown::HashMap::new();
        if !shape.disabled && old_handle != ColliderHandle::invalid() {
            let new_handle = collision_object.create_shape(shape, p_index, physics_engine);
            handle_mapping.insert(old_handle, new_handle);
            collision_object.get_mut_base().state.shapes[p_index].collider_handle = new_handle;
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[p_index],
                physics_engine,
            );
        }
        // Migrate monitored_objects for areas when handles change
        if let Some(area) = collision_object.get_mut_area() {
            // Store the handle mapping for later use in process_new_event
            area.state
                .handle_migration_map
                .extend(handle_mapping.iter().map(|(k, v)| (*k, *v)));
            let mut monitors_to_update: Vec<(
                (ColliderHandle, ColliderHandle),
                (ColliderHandle, ColliderHandle),
                MonitorInfo,
            )> = Vec::new();
            for ((other_handle, this_handle), monitor_info) in &area.state.monitored_objects {
                if let Some(&new_this_handle) = handle_mapping.get(this_handle) {
                    // This monitor's this_collider_handle was recreated, need to migrate it
                    monitors_to_update.push((
                        (*other_handle, *this_handle),
                        (*other_handle, new_this_handle),
                        *monitor_info,
                    ));
                }
            }
            // Remove old entries and insert with new handles
            for (old_key, new_key, monitor_info) in monitors_to_update {
                area.state.monitored_objects.remove(&old_key);
                area.state.monitored_objects.insert(new_key, monitor_info);
            }
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }

    pub(super) fn set_shape_transform(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_transform: Transform,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if p_index >= collision_object.get_base().state.shapes.len() {
            return;
        }
        collision_object.get_mut_base().state.shapes[p_index].xform = p_transform;
        let shape = &collision_object.get_base().state.shapes[p_index];
        collision_object
            .get_base()
            .update_shape_transform(shape, physics_engine);
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }

    pub(super) fn set_shape_disabled(
        collision_object: &mut dyn IRapierCollisionObject,
        p_index: usize,
        p_disabled: bool,
        physics_engine: &mut PhysicsEngine,
        physics_spaces: &mut PhysicsSpaces,
        physics_ids: &PhysicsIds,
    ) {
        if p_index >= collision_object.get_base().state.shapes.len() {
            return;
        }
        let shape = collision_object.get_base().state.shapes[p_index];
        if shape.disabled == p_disabled {
            return;
        }
        collision_object.get_mut_base().state.shapes[p_index].disabled = p_disabled;
        let shape = collision_object.get_base().state.shapes[p_index];
        if shape.disabled {
            collision_object.get_mut_base().state.shapes[p_index].collider_handle =
                collision_object.get_base().destroy_shape(
                    shape,
                    p_index,
                    physics_spaces,
                    physics_engine,
                    physics_ids,
                );
        }
        if !shape.disabled {
            collision_object.get_mut_base().state.shapes[p_index].collider_handle =
                collision_object.create_shape(shape, p_index, physics_engine);
            collision_object.get_base().update_shape_transform(
                &collision_object.get_base().state.shapes[p_index],
                physics_engine,
            );
        }
        if collision_object.get_base().is_space_valid() {
            collision_object.shapes_changed(physics_engine, physics_spaces, physics_ids);
        }
    }
}
