from typing import Tuple
from vast_invoke import Context, task
from common import (
    RESOURCEDIR,
    DOCKERDIR,
    FargateService,
    terraform_output,
)


@task(help={"tag": "The tag of the built image"})
def build_image(c, tag):
    """Build a tunnel image with the appropriate startup script"""
    c.run(
        f"""docker build \
            -f {DOCKERDIR}/tunnel.Dockerfile \
            -t {tag} \
            {RESOURCEDIR}"""
    )


def service_outputs(c: Context) -> Tuple[str, str, str]:
    cluster = terraform_output(c, "core-2", "fargate_cluster_name")
    family = terraform_output(c, "tunnel", "tunnel_task_family")
    service_name = terraform_output(c, "tunnel", "tunnel_service_name")
    return (cluster, service_name, family)


@task
def status(c):
    """Get the status of the tunnel service"""
    print(FargateService(*service_outputs(c)).get_task_status())


@task
def start(c):
    """Start the tunnel as an AWS Fargate task. Noop if it is already running"""
    FargateService(*service_outputs(c)).start_service()


@task
def stop(c):
    """Stop the tunnel instance and service"""
    FargateService(*service_outputs(c)).stop_service()


@task
def restart(c):
    """Stop the running tunnel task, the service starts a new one"""
    FargateService(*service_outputs(c)).restart_service()
