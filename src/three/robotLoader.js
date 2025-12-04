import URDFLoader from "urdf-loader";

export async function loadUR10e(scene) {
  const loader = new URDFLoader();
  // Map ROS package name to the public root so package://ur_description/... -> /meshes/ur10e/...
  loader.packages = { ur_description: "" };
  loader.workingPath = "/";
  loader.fetchOptions = { mode: "cors" };

  return new Promise((resolve, reject) => {
    loader.load(
      "/ur10e.urdf",
      (robot) => {
        // Align ROS Z-up into Three's Y-up (rotate -90deg about X).
        robot.rotation.set(-Math.PI / 2, 0, 0);
        robot.name = "ur10e";
        scene.add(robot);
        const jm = robot.joints || {};
        const keys = Object.keys(jm);
        if (keys.length === 0) {
          console.warn("UR10e joints not found on robot object");
        } else {
          console.info("UR10e joints detected:", keys);
        }
        const endEffector =
          robot.getObjectByName("wrist_3_link") || robot.getObjectByName("tool0");
        resolve({
          robot,
          joints: {
            j0: jm["shoulder_pan_joint"],
            j1: jm["shoulder_lift_joint"],
            j2: jm["elbow_joint"],
            j3: jm["wrist_1_joint"],
            j4: jm["wrist_2_joint"],
            j5: jm["wrist_3_joint"],
          },
          endEffector,
        });
      },
      undefined,
      (err) => {
        console.error("URDF load error", err);
        reject(err);
      },
    );
  });
}
