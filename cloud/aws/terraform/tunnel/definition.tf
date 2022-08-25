locals {
  container_definition = [
    {
      image     = var.tunnel_image
      name      = "main"
      essential = true
      # TODO make environement dynamic
      environment = [{
        name  = "CF_TUNNEL_ID"
        value = "12e80cc3-c4fa-4e3b-9f78-cb2eb798ddaa"
        }, {
        name  = "CF_SECRET"
        value = "eyJhIjoiYmMyNjllZjhjNzc4Njc0Nzg5Yzk2Mjc2YTQ1MGU0MmQiLCJ0IjoiMTJlODBjYzMtYzRmYS00ZTNiLTlmNzgtY2IyZWI3OThkZGFhIiwicyI6IllUVTFaRGhpTmpRdE5UUTJNaTAwTURBMExXSTVPRGt0TlRGaE5EUTVOamxpTURFNSJ9"
        }, {
        name  = "NAMESPACE_ID"
        value = "srv-575cmeiux4cb5ak5"
        }, {
        name  = "AWS_REGION"
        value = var.region_name
        }
      ]
      logConfiguration = {
        logDriver = "awslogs"
        options = {
          awslogs-group         = aws_cloudwatch_log_group.fargate_logging.name
          awslogs-region        = var.region_name
          awslogs-stream-prefix = "ecs"
        }
      }
    }
  ]
}
