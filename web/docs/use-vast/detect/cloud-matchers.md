---
sidebar_position: 1
---

# Cloud matchers

import CommercialPlugin from '@site/presets/CommercialPlugin.md';

<CommercialPlugin />

We provide a reference architecture and deployment scripts to have matchers running in the AWS cloud.

In order to deploy VAST in the AWS cloud with the Pro image, follow the steps described in the [deployment guide](/docs/setup-vast/deploy/aws-pro.md).

## Architecture

To deploy matchers in the cloud, we need to instantiate two main cloud resources:
- an SQS queue to reliable store and distribute the matched events
- a long running Fargate client that will attach to the VAST server and publish the matches to the queue

![AWS Architecture](https://user-images.githubusercontent.com/7913347/184834597-cc6ef751-2444-4741-aacf-f9f7fdb9482d.png)

## Setup

Once you have the base configuration deployed as described in the [deployment guide](/docs/setup-vast/deploy/aws-pro.md), you can deploy the matcher plugin:
```bash
./vast-cloud matcher.deploy -a
```

You can then create [VAST matchers](https://vast.io/docs/use-vast/detect/match-threat-intel#start-matchers) through the Lambda client:
```bash
./vast-cloud run-lambda -c "vast matcher start --mode=exact --match-types=addr feodo"
```
Similarly, you can load indicators into the created matchers.

We provide scripts that create and load matchers from external feeds such as the [Feodo Tracker](https://feodotracker.abuse.ch/):
```bash
./vast-cloud run-lambda -c file://$(pwd)/resources/scripts/matcher/feodo.sh
```
Note: `run-lambda` requires an absolute path when running a script from file.

Once the matchers are created, start the matcher client that will publish all matches to the managed queue:
```bash
./vast-cloud matcher.start-client
```

## Example usage

The matcher will trigger when data that contains the registered IoC is imported to VAST. We provide a flowlogs extract containing traffic that is currently flagged by the Feodo tracker in the test datasets:
```
./vast-cloud workbucket.deploy -a
./vast-cloud tests.import-data --dataset=flowlogs
```
Note: we also deploy the workbucket plugin as it is required by the `tests.import-data` command.

You can listen to matched events published on AWS SQS:
```
./vast-cloud matcher.attach
```
Note: Matched events are kept in the queue only for a few minutes.
