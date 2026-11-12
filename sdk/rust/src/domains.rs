use crate::{fields, identity, json, object, segment, Client, Error, Result, Value};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

pub struct Navigation<'a>(pub(crate) &'a Client);
pub struct Map<'a>(pub(crate) &'a Client);
pub struct Tasks<'a>(pub(crate) &'a Client);
pub struct Observations<'a>(pub(crate) &'a Client);
pub struct System<'a>(pub(crate) &'a Client);

impl Navigation<'_> {
    pub fn go(&self, x: f64, y: f64, z: f64, options: Value) -> Result<Value> {
        self.goal("/api/v1/goal", x, y, z, options)
    }
    pub fn go_to_click(&self, x: f64, y: f64, z: f64, options: Value) -> Result<Value> {
        self.goal("/api/v1/navigate/click", x, y, z, options)
    }
    fn goal(&self, path: &str, x: f64, y: f64, z: f64, options: Value) -> Result<Value> {
        if ![x, y, z].iter().all(|v| v.is_finite()) {
            return Err(Error::InvalidInput(
                "goal coordinates must be finite".into(),
            ));
        }
        self.0
            .command("POST", path, fields(options, json!({"x":x,"y":y,"z":z}))?)
    }
    pub fn select_planner(&self, name: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/navigation/planner",
            fields(options, json!({"name":name}))?,
        )
    }
    /// With a task ID, cancel exactly that task. Null cancels the current task.
    pub fn cancel(&self, task_id: Option<&str>, options: Value) -> Result<Value> {
        match task_id {
            Some(task_id) => self.control("cancel", task_id, options),
            None => self.0.command("POST", "/api/v1/navigation/cancel", options),
        }
    }
    pub fn pause(&self, task_id: &str, options: Value) -> Result<Value> {
        self.control("pause", task_id, options)
    }
    pub fn resume(&self, task_id: &str, options: Value) -> Result<Value> {
        self.control("resume", task_id, options)
    }
    pub fn set_speed_limit(
        &self,
        task_id: &str,
        max_speed_mps: f64,
        options: Value,
    ) -> Result<Value> {
        if !max_speed_mps.is_finite() {
            return Err(Error::InvalidInput("speed must be finite".into()));
        }
        self.control(
            "speed-limit",
            task_id,
            fields(options, json!({"max_speed_mps":max_speed_mps}))?,
        )
    }
    fn control(&self, action: &str, task_id: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            &format!("/api/v1/navigation/tasks/{}/{action}", segment(task_id)),
            fields(options, json!({"task_id":task_id}))?,
        )
    }
    pub fn status(&self) -> Result<Value> {
        self.0.get("/api/v1/navigation/status")
    }
    pub fn task(&self, task_id: &str) -> Result<Value> {
        self.0
            .get(&format!("/api/v1/navigation/tasks/{}", segment(task_id)))
    }
    pub fn goal_status(&self, request_id: &str) -> Result<Value> {
        self.0
            .get(&format!("/api/v1/navigation/goals/{}", segment(request_id)))
    }
    pub fn path(&self) -> Result<Value> {
        self.0.get("/api/v1/path")
    }
    pub fn start_exploration(&self) -> Result<Value> {
        self.0.command("POST", "/api/v1/explore/start", Value::Null)
    }
    pub fn stop_exploration(&self) -> Result<Value> {
        self.0.command("POST", "/api/v1/explore/stop", Value::Null)
    }
    pub fn exploration_status(&self) -> Result<Value> {
        self.0.get("/api/v1/explore/status")
    }
    pub fn localization_status(&self) -> Result<Value> {
        self.0.get("/api/v1/localization/status")
    }
    pub fn relocalize(&self, map_name: &str, initial_pose: Value, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/localization/relocalizations",
            fields(
                options,
                json!({"map_name":map_name,"mode":"seeded","initial_pose":initial_pose}),
            )?,
        )
    }
    pub fn global_relocalize(&self, map_name: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/localization/relocalizations",
            fields(options, json!({"map_name":map_name,"mode":"global"}))?,
        )
    }
    pub fn start_map_tracking(&self, map_name: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/localization/map-tracking",
            fields(options, json!({"map_name":map_name}))?,
        )
    }
    pub fn position(&self) -> Result<Value> {
        let state = self.0.system().state()?;
        let position = &state["localization"]["odometry"];
        if position["frame_id"] != "map" {
            return Err(Error::Operation(state));
        }
        Ok(position.clone())
    }
    /// Poll the identified task ledger. Stopping this wait does not cancel the task.
    pub fn wait_until_arrived(
        &self,
        task_id: &str,
        timeout: Duration,
        poll_interval: Duration,
    ) -> Result<Value> {
        if task_id.is_empty() || poll_interval.is_zero() {
            return Err(Error::InvalidInput(
                "task_id and positive poll interval are required".into(),
            ));
        }
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let payload = self.task(task_id)?;
            let status = &payload["status"];
            let observed_id = status["task_id"]
                .as_str()
                .or_else(|| payload["task_id"].as_str());
            if payload["found"] == true && observed_id == Some(task_id) {
                match status["lifecycle_state_name"]
                    .as_str()
                    .or_else(|| status["state_name"].as_str())
                    .unwrap_or("")
                {
                    "SUCCESS" => return Ok(payload),
                    "FAILED" | "CANCELLED" => return Err(Error::Operation(payload)),
                    _ => {}
                }
            }
            std::thread::sleep(
                poll_interval.min(deadline.saturating_duration_since(Instant::now())),
            );
        }
        Err(Error::Timeout(format!(
            "navigation task {task_id} did not complete"
        )))
    }
}

impl Map<'_> {
    pub fn list(&self) -> Result<Value> {
        self.0.get("/api/v1/slam/maps")
    }
    pub fn save(&self, options: Value) -> Result<Value> {
        self.0.command("POST", "/api/v1/map/save", options)
    }
    pub fn operations(&self, limit: usize) -> Result<Value> {
        self.0.request(
            "GET",
            "/api/v1/maps/operations",
            &[("limit", &limit.to_string())],
            None,
        )
    }
    pub fn operation(&self, operation_id: &str) -> Result<Value> {
        self.0.get(&format!(
            "/api/v1/maps/operations/{}",
            segment(operation_id)
        ))
    }
    pub fn cancel_operation(&self, operation_id: &str) -> Result<Value> {
        self.operation_command(operation_id, "cancel", Value::Null)
    }
    pub fn retry_operation(&self, operation_id: &str) -> Result<Value> {
        self.operation_command(operation_id, "retry", Value::Null)
    }
    pub fn process_operation(&self, operation_id: &str) -> Result<Value> {
        self.operation_command(operation_id, "process", Value::Null)
    }
    pub fn reoptimize_operation(&self, operation_id: &str, options: Value) -> Result<Value> {
        self.operation_command(operation_id, "reoptimize", options)
    }
    fn operation_command(&self, operation_id: &str, action: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            &format!("/api/v1/maps/operations/{}/{action}", segment(operation_id)),
            options,
        )
    }
    pub fn download_pcd(&self, name: &str, target: impl AsRef<Path>) -> Result<PathBuf> {
        self.0
            .download(&format!("/api/v1/maps/{}/pcd", segment(name)), target)
    }
    pub fn download_octomap(&self, name: &str, target: impl AsRef<Path>) -> Result<PathBuf> {
        self.0
            .download(&format!("/api/v1/maps/{}/octomap", segment(name)), target)
    }
    pub fn download_source(&self, operation_id: &str, target: impl AsRef<Path>) -> Result<PathBuf> {
        self.0.download(
            &format!(
                "/api/v1/maps/operations/{}/source/pcd",
                segment(operation_id)
            ),
            target,
        )
    }
    pub fn rename(&self, old_name: &str, new_name: &str) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/map/rename",
            json!({"old_name":old_name,"new_name":new_name}),
        )
    }
    pub fn delete(&self, name: &str) -> Result<Value> {
        self.0.request(
            "DELETE",
            &format!("/api/v1/maps/{}", segment(name)),
            &[],
            None,
        )
    }
    /// source_path is a path in the Gateway's configured import directory.
    pub fn import_pcd(&self, name: &str, source_path: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/maps/import_pcd",
            fields(options, json!({"name":name,"source_path":source_path}))?,
        )
    }
    pub fn reset_cloud(&self) -> Result<Value> {
        self.0
            .command("POST", "/api/v1/map_cloud/reset", Value::Null)
    }
    pub fn points(&self) -> Result<Value> {
        self.0.get("/api/v1/map/points")
    }
    pub fn saved_points(&self, name: &str) -> Result<Value> {
        self.0
            .get(&format!("/api/v1/maps/{}/points", segment(name)))
    }
    pub fn surface(&self, name: &str, query: &[(&str, &str)]) -> Result<Value> {
        self.0.request(
            "GET",
            &format!("/api/v1/maps/{}/surface", segment(name)),
            query,
            None,
        )
    }
    pub fn no_go(&self, name: &str) -> Result<Value> {
        self.0.get(&format!("/api/v1/maps/{}/no_go", segment(name)))
    }
    pub fn set_no_go(
        &self,
        name: &str,
        region: Value,
        expected_content_epoch: u64,
    ) -> Result<Value> {
        self.0.request(
            "PUT",
            &format!("/api/v1/maps/{}/no_go", segment(name)),
            &[],
            Some(&json!({"region":region,"expected_content_epoch":expected_content_epoch})),
        )
    }
    pub fn delete_no_go(
        &self,
        name: &str,
        region_id: &str,
        expected_content_epoch: u64,
    ) -> Result<Value> {
        self.0.request(
            "DELETE",
            &format!(
                "/api/v1/maps/{}/no_go/{}",
                segment(name),
                segment(region_id)
            ),
            &[(
                "expected_content_epoch",
                &expected_content_epoch.to_string(),
            )],
            None,
        )
    }
    pub fn preview_edits(&self, name: &str, options: Value) -> Result<Value> {
        self.map_command(name, "edit_preview", options)
    }
    pub fn save_as(&self, name: &str, options: Value) -> Result<Value> {
        self.map_command(name, "save_as", options)
    }
    pub fn crop(&self, name: &str, options: Value) -> Result<Value> {
        self.map_command(name, "crop", options)
    }
    pub fn mark_zone(&self, name: &str, options: Value) -> Result<Value> {
        self.map_command(name, "mark_zone", options)
    }
    pub fn build_occupancy(&self, name: &str) -> Result<Value> {
        self.map_command(name, "build_occupancy", Value::Null)
    }
    pub fn build_octomap(&self, name: &str, options: Value) -> Result<Value> {
        self.map_command(name, "build_octomap", options)
    }
    pub fn validate_plan(&self, name: &str, options: Value) -> Result<Value> {
        self.0.request(
            "POST",
            &format!("/api/v1/maps/{}/validate_plan", segment(name)),
            &[],
            Some(&Value::Object(object(options)?)),
        )
    }
    pub fn edit_voxels(&self, name: &str, options: Value) -> Result<Value> {
        self.map_command(name, "voxels/edit", options)
    }
    pub fn voxel_edits(&self, name: &str) -> Result<Value> {
        self.0
            .get(&format!("/api/v1/maps/{}/voxels/edits", segment(name)))
    }
    fn map_command(&self, name: &str, action: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            &format!("/api/v1/maps/{}/{action}", segment(name)),
            options,
        )
    }
    pub fn locations(&self) -> Result<Value> {
        self.0.get("/api/v1/locations")
    }
    pub fn tag_location(&self, name: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/locations",
            fields(options, json!({"name":name}))?,
        )
    }
    pub fn delete_location(&self, name: &str) -> Result<Value> {
        self.0.request(
            "DELETE",
            &format!("/api/v1/locations/{}", segment(name)),
            &[],
            None,
        )
    }
    /// Submit once, then poll durable operation state without resubmitting on connection errors.
    pub fn save_and_wait(
        &self,
        options: Value,
        timeout: Duration,
        poll_interval: Duration,
        wait_for_source: bool,
    ) -> Result<Value> {
        if poll_interval.is_zero() {
            return Err(Error::InvalidInput(
                "positive poll interval is required".into(),
            ));
        }
        let deadline = Instant::now() + timeout;
        let source_only = wait_for_source || options["auto_process"] == false;
        let mut options = object(options)?;
        let request_id = options
            .entry("request_id")
            .or_insert_with(|| json!(identity()))
            .clone();
        let receipt = self.save(Value::Object(options))?;
        if receipt["ok"] == false
            || receipt["accepted"] == false
            || receipt["request_id"] != request_id
        {
            return Err(Error::Operation(receipt));
        }
        let operation_id = receipt["operation_id"]
            .as_str()
            .or_else(|| receipt["operation"]["operation_id"].as_str())
            .filter(|id| !id.is_empty())
            .ok_or_else(|| Error::Operation(receipt.clone()))?;
        if receipt["operation"]["operation_id"]
            .as_str()
            .is_some_and(|id| id != operation_id)
            || receipt["operation"]["request_id"]
                .as_str()
                .is_some_and(|id| Some(id) != request_id.as_str())
        {
            return Err(Error::Operation(receipt));
        }
        while Instant::now() < deadline {
            let payload = self.operation(operation_id)?;
            let operation = &payload["operation"];
            if [&payload["operation_id"], &operation["operation_id"]]
                .iter()
                .any(|value| value.as_str().is_some_and(|id| id != operation_id))
            {
                return Err(Error::Operation(payload));
            }
            if source_only && operation["source_saved"] == true {
                return Ok(payload);
            }
            match operation["state"].as_str().unwrap_or("") {
                "SUCCEEDED" => return Ok(payload),
                "FAILED" | "CANCELLED" | "CANCELED" => return Err(Error::Operation(payload)),
                _ => {}
            }
            if payload["ok"] == false || payload["success"] == false {
                return Err(Error::Operation(payload));
            }
            std::thread::sleep(
                poll_interval.min(deadline.saturating_duration_since(Instant::now())),
            );
        }
        Err(Error::Timeout(format!(
            "map operation {operation_id} did not complete"
        )))
    }
}

impl Tasks<'_> {
    pub fn send_instruction(&self, text: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/instruction",
            fields(options, json!({"text":text}))?,
        )
    }
    pub fn get_agent_run(&self, run_id: &str) -> Result<Value> {
        self.0
            .get(&format!("/api/v1/agent/runs/{}", segment(run_id)))
    }
    pub fn cancel_agent_run(&self, run_id: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            &format!("/api/v1/agent/runs/{}/cancel", segment(run_id)),
            options,
        )
    }
    pub fn start_follow(&self, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/visual_servo",
            fields(options, json!({"mode":"follow"}))?,
        )
    }
    pub fn get_follow_task(&self, follow_task_id: &str, product_session_id: &str) -> Result<Value> {
        self.0.request(
            "GET",
            &format!("/api/v1/follow/{}", segment(follow_task_id)),
            &[("product_session_id", product_session_id)],
            None,
        )
    }
    pub fn cancel_follow_task(
        &self,
        follow_task_id: &str,
        product_session_id: &str,
        options: Value,
    ) -> Result<Value> {
        self.0.command(
            "POST",
            &format!("/api/v1/follow/{}/cancel", segment(follow_task_id)),
            fields(options, json!({"product_session_id":product_session_id}))?,
        )
    }
    pub fn inspection_routes(&self, map_id: Option<&str>) -> Result<Value> {
        self.0.request(
            "GET",
            "/api/v1/inspection/routes",
            &map_id.map(|id| vec![("map_id", id)]).unwrap_or_default(),
            None,
        )
    }
    pub fn inspection_route(&self, route_id: &str, map_id: Option<&str>) -> Result<Value> {
        self.0.request(
            "GET",
            &format!("/api/v1/inspection/routes/{}", segment(route_id)),
            &map_id.map(|id| vec![("map_id", id)]).unwrap_or_default(),
            None,
        )
    }
    pub fn save_inspection_route(&self, route: Value) -> Result<Value> {
        self.0
            .request("POST", "/api/v1/inspection/routes", &[], Some(&route))
    }
    pub fn delete_inspection_route(&self, route_id: &str, map_id: Option<&str>) -> Result<Value> {
        self.0.request(
            "DELETE",
            &format!("/api/v1/inspection/routes/{}", segment(route_id)),
            &map_id.map(|id| vec![("map_id", id)]).unwrap_or_default(),
            None,
        )
    }
    pub fn inspection_tasks(&self, query: &[(&str, &str)]) -> Result<Value> {
        self.0
            .request("GET", "/api/v1/inspection/tasks", query, None)
    }
    pub fn list_inspection_evidence(&self, limit: usize) -> Result<Value> {
        self.0.request(
            "GET",
            "/api/v1/inspection/evidence",
            &[("limit", &limit.to_string())],
            None,
        )
    }
    pub fn inspection_evidence(&self, evidence_id: &str) -> Result<Value> {
        self.0.get(&format!(
            "/api/v1/inspection/evidence/{}",
            segment(evidence_id)
        ))
    }
    pub fn review_inspection_evidence(
        &self,
        evidence_id: &str,
        verdict: &str,
        note: &str,
    ) -> Result<Value> {
        self.0.request(
            "POST",
            &format!(
                "/api/v1/inspection/evidence/{}/review",
                segment(evidence_id)
            ),
            &[],
            Some(&json!({"verdict": verdict, "note": note})),
        )
    }
    pub fn download_inspection_artifact(
        &self,
        evidence_id: &str,
        kind: &str,
        target: impl AsRef<Path>,
    ) -> Result<PathBuf> {
        self.0.download(
            &format!(
                "/api/v1/inspection/evidence/{}/artifacts/{}",
                segment(evidence_id),
                segment(kind)
            ),
            target,
        )
    }
    pub fn download_inspection_measurement(
        &self,
        evidence_id: &str,
        sensor_id: &str,
        target: impl AsRef<Path>,
    ) -> Result<PathBuf> {
        self.0.download(
            &format!(
                "/api/v1/inspection/evidence/{}/measurements/{}",
                segment(evidence_id),
                segment(sensor_id)
            ),
            target,
        )
    }
    pub fn start_inspection(&self, route_id: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/inspection/tasks",
            fields(options, json!({"route_id":route_id}))?,
        )
    }
    pub fn inspection_task(&self, task_id: &str) -> Result<Value> {
        self.0
            .get(&format!("/api/v1/inspection/tasks/{}", segment(task_id)))
    }
    pub fn pause_inspection(&self, task_id: &str, options: Value) -> Result<Value> {
        self.inspection_control(task_id, "pause", options)
    }
    pub fn resume_inspection(&self, task_id: &str, options: Value) -> Result<Value> {
        self.inspection_control(task_id, "resume", options)
    }
    pub fn cancel_inspection(&self, task_id: &str, options: Value) -> Result<Value> {
        self.inspection_control(task_id, "cancel", options)
    }
    fn inspection_control(&self, task_id: &str, action: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            &format!("/api/v1/inspection/tasks/{}/{action}", segment(task_id)),
            options,
        )
    }
    pub fn inspection_report(&self, task_id: &str) -> Result<Value> {
        self.0.get(&format!(
            "/api/v1/inspection/tasks/{}/report",
            segment(task_id)
        ))
    }
}

impl Observations<'_> {
    pub fn scene(&self) -> Result<Value> {
        self.0.get("/api/v1/scene_graph")
    }
    pub fn camera_snapshot(&self) -> Result<Vec<u8>> {
        self.0.bytes("/api/v1/camera/snapshot", &[])
    }
    pub fn camera_status(&self) -> Result<Value> {
        self.0.get("/api/v1/camera/status")
    }
    pub fn media_status(&self) -> Result<Value> {
        self.0.get("/api/v1/webrtc/status")
    }
    pub fn camera_diagnostics(&self) -> Result<Value> {
        self.0.get("/api/v1/camera/diagnostics")
    }
    pub fn temporal_memory(&self, query: &[(&str, &str)]) -> Result<Value> {
        self.0
            .request("GET", "/api/v1/memory/temporal", query, None)
    }
    pub fn search_memory(&self, _query: &str) -> Result<Value> {
        Err(Error::Unsupported(
            concat!(
                "The Gateway requires an embedding and provides no text encoder. ",
                "Use search_memory_embedding with the observation store's embedding model."
            )
            .into(),
        ))
    }
    pub fn search_memory_embedding(&self, embedding: &[f64], options: Value) -> Result<Value> {
        if !embedding.iter().all(|value| value.is_finite()) {
            return Err(Error::InvalidInput(
                "embedding values must be finite".into(),
            ));
        }
        self.0.request(
            "POST",
            "/api/v1/memory/temporal/semantic",
            &[],
            Some(&fields(options, json!({"embedding":embedding}))?),
        )
    }
    pub fn start_recording(&self, options: Value) -> Result<Value> {
        self.0.command("POST", "/api/v1/recordings/start", options)
    }
    pub fn stop_recording(&self) -> Result<Value> {
        self.0
            .command("POST", "/api/v1/recordings/stop", Value::Null)
    }
    pub fn stop_recording_session(&self, expected_session_id: &str) -> Result<Value> {
        self.0.request(
            "POST",
            "/api/v1/recordings/stop",
            &[],
            Some(&json!({"expected_session_id": expected_session_id})),
        )
    }
    pub fn recording_status(&self) -> Result<Value> {
        self.0.get("/api/v1/recordings/status")
    }
    pub fn recordings(&self, limit: usize) -> Result<Value> {
        self.0.request(
            "GET",
            "/api/v1/recordings",
            &[("limit", &limit.to_string())],
            None,
        )
    }
    pub fn recording(&self, session_id: &str) -> Result<Value> {
        self.0
            .get(&format!("/api/v1/recordings/{}", segment(session_id)))
    }
    pub fn recording_catalog(&self) -> Result<Value> {
        self.0.get("/api/v1/recordings/catalog")
    }
    pub fn recording_streams(&self) -> Result<Value> {
        self.0.get("/api/v1/recordings/streams")
    }
    pub fn download_recording_file(
        &self,
        session_id: &str,
        artifact_path: &str,
        target: impl AsRef<Path>,
    ) -> Result<PathBuf> {
        let artifact = artifact_path
            .split('/')
            .map(segment)
            .collect::<Vec<_>>()
            .join("/");
        self.0.download(
            &format!(
                "/api/v1/recordings/{}/files/{artifact}",
                segment(session_id)
            ),
            target,
        )
    }
    pub fn download_recording_archive(
        &self,
        session_id: &str,
        target: impl AsRef<Path>,
    ) -> Result<PathBuf> {
        self.0.download(
            &format!("/api/v1/recordings/{}/archive", segment(session_id)),
            target,
        )
    }
    pub fn recording_summary(&self, session_id: &str) -> Result<Value> {
        self.0.get(&format!(
            "/api/v1/recordings/{}/summary",
            segment(session_id)
        ))
    }
    pub fn recording_series(
        &self,
        session_id: &str,
        topic: &str,
        query: &[(&str, &str)],
    ) -> Result<Value> {
        let mut query = query.to_vec();
        query.retain(|(key, _)| *key != "topic");
        query.push(("topic", topic));
        self.0.request(
            "GET",
            &format!("/api/v1/recordings/{}/series", segment(session_id)),
            &query,
            None,
        )
    }
    pub fn recording_scene_info(&self, session_id: &str) -> Result<Value> {
        self.0.get(&format!(
            "/api/v1/recordings/{}/scene-info",
            segment(session_id)
        ))
    }
    pub fn recording_scene(&self, session_id: &str, query: &[(&str, &str)]) -> Result<Value> {
        self.0.request(
            "GET",
            &format!("/api/v1/recordings/{}/scene", segment(session_id)),
            query,
            None,
        )
    }
    pub fn recording_camera_frame(
        &self,
        session_id: &str,
        time_s: f64,
        segment_index: usize,
        query: &[(&str, &str)],
    ) -> Result<Vec<u8>> {
        if !time_s.is_finite() {
            return Err(Error::InvalidInput("frame time must be finite".into()));
        }
        let time = time_s.to_string();
        let index = segment_index.to_string();
        let mut query = query.to_vec();
        query.retain(|(key, _)| !matches!(*key, "t" | "segment"));
        query.extend([("t", time.as_str()), ("segment", index.as_str())]);
        self.0.bytes(
            &format!("/api/v1/recordings/{}/camera-frame", segment(session_id)),
            &query,
        )
    }
    pub fn validate_recording(&self, session_id: &str) -> Result<Value> {
        self.0.request(
            "POST",
            &format!("/api/v1/recordings/{}/validate", segment(session_id)),
            &[],
            None,
        )
    }
    pub fn delete_recording(&self, session_id: &str) -> Result<Value> {
        self.0.request(
            "DELETE",
            &format!("/api/v1/recordings/{}", segment(session_id)),
            &[],
            None,
        )
    }
}

impl System<'_> {
    pub fn state(&self) -> Result<Value> {
        self.0.get("/api/v1/state")
    }
    pub fn health(&self) -> Result<Value> {
        self.0.get("/api/v1/health")
    }
    pub fn session(&self) -> Result<Value> {
        self.0.get("/api/v1/session")
    }
    pub fn product_status(&self) -> Result<Value> {
        self.0.get("/api/v1/product-control")
    }
    pub fn product_operation(&self, request_id: &str) -> Result<Value> {
        self.0.get(&format!(
            "/api/v1/product-control/operations/{}",
            segment(request_id)
        ))
    }
    pub fn stop_product(&self, expected_product_session_id: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/product-control/stop",
            fields(
                options,
                json!({"expected_product_session_id": expected_product_session_id}),
            )?,
        )
    }
    pub fn switch_product(
        &self,
        product: &str,
        expected_product_session_id: &str,
        options: Value,
    ) -> Result<Value> {
        self.0.command("POST", "/api/v1/product-control/switch", fields(options,
            json!({"product": product, "expected_product_session_id": expected_product_session_id}))?)
    }
    pub fn set_mode(&self, mode: &str) -> Result<Value> {
        self.0.command("POST", "/api/v1/mode", json!({"mode":mode}))
    }
    pub fn capabilities(&self) -> Result<Value> {
        self.0.get("/api/v1/app/capabilities")
    }
    pub fn bootstrap(&self) -> Result<Value> {
        self.0.get("/api/v1/app/bootstrap")
    }
    pub fn readiness(&self) -> Result<Value> {
        self.0.get("/api/v1/readiness")
    }
    pub fn field_check(&self) -> Result<Value> {
        self.0
            .command("POST", "/api/v1/diagnostics/field-check", Value::Null)
    }
    pub fn runtime_contract(&self) -> Result<Value> {
        self.0.get("/api/v1/diagnostics/runtime-contract")
    }
    pub fn slam_status(&self) -> Result<Value> {
        self.0.get("/api/v1/slam/status")
    }
    pub fn emergency_stop(&self) -> Result<Value> {
        self.0.command("POST", "/api/v1/stop", Value::Null)
    }
    pub fn clear_emergency_stop(&self, options: Value) -> Result<Value> {
        self.0.command("POST", "/api/v1/estop/reset", options)
    }
    pub fn stop_motion(&self, options: Value) -> Result<Value> {
        self.0.command("POST", "/api/v1/motion/stop", options)
    }
    pub fn resume_motion(&self, options: Value) -> Result<Value> {
        self.0.command("POST", "/api/v1/navigation/resume", options)
    }
    pub fn stand_up(&self, options: Value) -> Result<Value> {
        self.0.command("POST", "/api/v1/motion/stand", options)
    }
    pub fn sit_down(&self, options: Value) -> Result<Value> {
        self.0.command("POST", "/api/v1/motion/sit", options)
    }
    pub fn enable_motors(&self, options: Value) -> Result<Value> {
        self.0
            .command("POST", "/api/v1/motion/motors/enable", options)
    }
    pub fn disable_motors(&self, options: Value) -> Result<Value> {
        self.0
            .command("POST", "/api/v1/motion/motors/disable", options)
    }
    pub fn switch_profile(&self, name: &str, options: Value) -> Result<Value> {
        self.0.command(
            "POST",
            "/api/v1/motion/profile",
            fields(options, json!({"name":name}))?,
        )
    }
    pub fn acquire_lease(&self, client_id: &str, ttl: f64) -> Result<Value> {
        self.lease("acquire", client_id, Some(ttl))
    }
    pub fn renew_lease(&self, client_id: &str, ttl: f64) -> Result<Value> {
        self.lease("renew", client_id, Some(ttl))
    }
    pub fn release_lease(&self, client_id: &str) -> Result<Value> {
        self.lease("release", client_id, None)
    }
    fn lease(&self, action: &str, client_id: &str, ttl: Option<f64>) -> Result<Value> {
        let mut body = json!({"action":action,"client_id":client_id});
        if let Some(ttl) = ttl {
            if !ttl.is_finite() {
                return Err(Error::InvalidInput("lease ttl must be finite".into()));
            }
            body["ttl"] = json!(ttl);
        }
        self.0.command("POST", "/api/v1/lease", body)
    }
}
