from __future__ import annotations

import argparse
import json
from pathlib import Path

from armctrl.agent_simulation import (
    AgentCliSimulationExperiment,
    AgentCliSimulationExperimentRequest,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a fixed non-hardware Agent CLI simulation experiment."
    )
    parser.add_argument("--output", default="runs/agent-cli-sim-experiment")
    parser.add_argument("--json", action="store_true", dest="as_json")
    args = parser.parse_args(argv)

    summary = AgentCliSimulationExperiment().run(
        AgentCliSimulationExperimentRequest(output_dir=Path(args.output))
    )
    if args.as_json:
        print(json.dumps(summary, ensure_ascii=False))
    else:
        print(f"acceptance_status: {summary['acceptance_status']}")
        print(f"output_dir: {summary['output_dir']}")
        print(f"summary: {summary['artifacts']['summary']}")
    return 0 if summary["acceptance_status"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
