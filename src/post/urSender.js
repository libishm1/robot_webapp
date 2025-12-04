import fs from "node:fs";
import net from "node:net";

export function sendToUR(ip, script) {
  return new Promise((resolve, reject) => {
    const client = new net.Socket();

    client.on("error", reject);
    client.on("close", resolve);

    client.connect(30002, ip, () => {
      client.write(script);
      client.end();
    });
  });
}

if (import.meta.url === `file://${process.argv[1]}`) {
  const [, , ip, scriptFile] = process.argv;
  if (!ip || !scriptFile) {
    console.error("Usage: node src/post/urSender.js <robot_ip> <script.urscript>");
    process.exit(1);
  }

  const script = fs.readFileSync(scriptFile, "utf-8");
  sendToUR(ip, script)
    .then(() => console.log(`Sent URScript to ${ip}:30002`))
    .catch((err) => {
      console.error("Failed to send URScript:", err);
      process.exitCode = 1;
    });
}
