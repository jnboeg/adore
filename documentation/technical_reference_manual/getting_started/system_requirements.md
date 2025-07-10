# System Requirements
The following section will detail the recommended hardware/system configuration,
as well as, required software configuration in order to build and run ADORe and
it's components.

### Minimum System Configurations 
**CPU:** 

- Intel Core i7 7700K or equivalent/better
- The more cores you have, the more trajectory planners you can run in 
  parallel.
- No specific graphics card is required as everything (except plotting) runs on 
  the CPU
- Any x86 base equivalent processor (ARM support is planned)

**RAM:** 

Min 8GB for execution. Compilation process is faster with 16+GB

**HD storage:**

- at least 2.5 GB to clone the repository
- at least 15 GB to build all necessary docker context

**Operating system:** 

- Anything that supports newer docker versions. 
- Recommended: Ubuntu 20.04, 22.04, 24.04

**Network:**

  A reliable network with high throughput and low latency. Initial 
  build can take a significant amount of time to pull all necessary dependencies
  from apt and docker. A poor connection will result in non-deterministic build 
  failures. 

### Software Requirements

- [Docker 🔗](https://www.docker.com/) v20.10.17 or greater and docker compose v2.6.0 or greater. To install
  the latest docker and docker compose run the following command:
```bash
curl -sSL https://raw.githubusercontent.com/DLR-TS/adore_tools/master/tools/install_docker.sh | bash`. 
```
For more information review the official docker documentation: [https://docs.docker.com/engine/install/ubuntu/ 🔗](https://docs.docker.com/engine/install/ubuntu/)

---
> **⚠️ WARNING:**
> As a general rule you should never run shell scripts from untrusted sources. 
---

- [GNU Make 🔗](https://www.gnu.org/software/make/) is the backbone of the ADORe build system and is also required.
