# CI/CD

## Continuous integration

`Validate controller` runs on pushes and pull requests using Python 3.11 and 3.13
and Node 22. It checks every Python source, extracts/checks the embedded browser
JavaScript without importing the hardware host, runs unit tests, and publishes
an installable source artifact. Actions are pinned to reviewed commit hashes.
No robot, camera, Tailscale credentials, or Python camera packages are required.

Run the same checks locally with `python3 scripts/check.py` (Node must be installed).

## Manual continuous deployment

`Deploy host UI (manual)` runs checks, joins Tailscale, and installs host code over
SSH only after an operator dispatches it. It refuses a busy controller. It
restarts `arm-control.service`, then checks `/state`. It does not flash firmware,
apply the Klipper patch, restart Klipper, change boundaries/calibration, home or
move the robot, overwrite programs/feature settings, or install dependencies.
It creates no backups or automatic regeneration. If health fails, the workflow
fails and leaves the uploaded code for inspection; no automatic rollback occurs.

Create a GitHub environment named `controller` (optional required reviewers) and
configure these environment/repository secrets before the first deployment:

| Secret | Purpose |
|---|---|
| TS_OAUTH_CLIENT_ID | Tailscale OAuth client allowed to create `tag:ci` nodes |
| TS_OAUTH_SECRET | Its secret; restrict tailnet policy to this controller |
| CONTROLLER_TARGET | `linux-user@tailnet-hostname` reachable by the CI node |
| CONTROLLER_SSH_KEY | Dedicated private deployment SSH key |
| CONTROLLER_KNOWN_HOSTS | Verified SSH host-key entry for that hostname |

Do not disable host-key verification. Authorize the matching SSH public key on
the controller. Tailscale policy must allow `tag:ci` to SSH to this machine.
The host needs an existing `~/arm-control/venv`, desired dependencies, a prepared
`features.json`, matching installed Klipper extension/geometry, and the enabled
`arm-control.service`. Provide narrowly scoped passwordless permission for
`/usr/bin/systemctl restart arm-control`; the existing Klipper-only permission
does not allow this. Test with `sudo -n -l /usr/bin/systemctl restart arm-control`.

Secrets/admin policy are deliberately not embedded in the repository. The
workflow is complete but cannot perform a deployment until these are supplied.
Use an Actions run to download the source artifact for manual installation if
automated deployment credentials are not desired.

The optional feature changes are published for review, not silently deployed to
the already-tested reference computer. For its first update, explicitly prepare
`features.json` with camera/remote enabled and keep rail usage disabled, matching
its running configuration.
