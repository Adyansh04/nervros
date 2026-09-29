//! Commands that talk to a live ROS graph.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use anyhow::{Context as _, Result, bail};
use nervros_core::look::{LookTool, SnapshotStore};
use nervros_core::profile::{Profile, Transport};
use nervros_core::tools::{Status, Tool as _};
use nervros_ros::{R2rConfig, R2rPort, RobotPort};

/// Starts the node after checking the environment matches the profile; the environment decides,
/// because ROS reads it when the node starts.
pub(crate) fn connect(profile: &Profile) -> Result<Arc<dyn RobotPort>> {
    let domain = std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|d| d.parse::<u32>().ok())
        .unwrap_or(0);
    if domain != profile.ros.domain_id {
        bail!(
            "ROS_DOMAIN_ID is {domain} but the profile says {}; export it before starting",
            profile.ros.domain_id
        );
    }
    let udp = std::env::var("FASTDDS_BUILTIN_TRANSPORTS").is_ok_and(|t| t == "UDPv4");
    if profile.ros.transport == Transport::Udp && !udp {
        bail!(
            "the profile wants UDP only; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 (NERVROS_UDP_ONLY=1 with scripts/ros-env.sh)"
        );
    }
    let config = R2rConfig {
        node_name: profile.ros.node_name.clone(),
        ..R2rConfig::default()
    };
    Ok(Arc::new(
        R2rPort::start(config).context("starting the ROS node")?,
    ))
}

pub(crate) async fn look(profile_path: &Path, out: &Path) -> Result<()> {
    let profile = Profile::load(profile_path).context("loading the profile")?;
    let config = profile
        .look
        .clone()
        .context("the profile has no [look] section")?;
    let robot = connect(&profile)?;
    let tool = LookTool::start(config, robot, Arc::new(SnapshotStore::default()))
        .context("subscribing to the camera")?;
    // Frames and discovery need a moment after the node starts.
    tokio::time::sleep(Duration::from_secs(2)).await;
    let outcome = tool.call(serde_json::json!({})).await;
    if outcome.status != Status::Succeeded {
        bail!("look failed: {}", outcome.message);
    }
    println!("{}", serde_json::to_string_pretty(&outcome.data)?);
    if let Some(image) = outcome.images.first() {
        std::fs::write(out, image.jpeg.as_slice())
            .with_context(|| format!("writing {}", out.display()))?;
        eprintln!(
            "marked image: {} ({}x{})",
            out.display(),
            image.width,
            image.height
        );
    }
    Ok(())
}
