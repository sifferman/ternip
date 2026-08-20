<div align="center">
  <img src="docs/ternip.svg" alt="Ternip Logo" width="200"/>
  <h1>Ternip</h1>
  <p>RTL accelerator for Ridger Chu's <a href="https://github.com/ridgerchu/matmulfreellm">MatmulFree LLM</a></p>
</div>

---

## Overview

Ternip is a parameterizable, open-source RTL implementation of a hardware accelerator targeting the MatmulFree LLM algoirhtnm. MatmulFree LLMs replace traditional matrix multiplication with ternary weight operations, enabling significant reductions in compute and memory bandwidth — making them well-suited for hardware acceleration.

This project is licensed under the [BSD 3-Clause License](LICENSE) and is free to use, modify, and distribute.

### Notable Files

| File | Description |
|------|-------------|
| [rtl/ternip_pkg.sv](rtl/ternip_pkg.sv) | Base configuration struct (`ternip_cfg_t`) |
| [rtl/ternip_types.sv](rtl/ternip_types.sv) | Config-derived values and data types (`ternip_types#(Cfg)`) |
| [rtl/ternip/ternip_core.sv](rtl/ternip/ternip_core.sv) | Top-level compute core |
| [rtl/fus/ternip_tmatmul.sv](rtl/fus/ternip_tmatmul.sv) | Ternary matrix multiplication unit |
| [rtl/fus/ternip_rms.sv](rtl/fus/ternip_rms.sv) | RMS normalization unit |
| [rtl/math/](rtl/math/) | Math modules (sqrt, sigmoid, SiLU, and more) |

Configuration is passed as a threaded `ternip_pkg::ternip_cfg_t` struct
parameter (`Cfg`) rather than a global config file.

*Tests and build flow are not currently provided but will be made available shortly.*

## Dependencies

Ternip depends on the [BaseJump STL](https://github.com/bespoke-silicon-group/basejump_stl) hardware library.

## FuseSoC

Ternip is available on the [FuseSoC Package Directory](https://fusesoc.net). To add it as a library using the GitHub repo directly:

```bash
fusesoc library add ternip https://github.com/sifferman/ternip --sync-type=git
```

Then declare it as a dependency in your `.core` file:

```yaml
filesets:
  rtl:
    depend:
      - sifferman::ternip
```

## Citation

If you use this work, please cite the original MatmulFree LLM paper:

```bibtex
@article{zhu2024scalable,
  title   = {Scalable MatMul-free Language Modeling},
  author  = {Zhu, Rui-Jie and Zhang, Yu and Sifferman, Ethan and Sheaves, Tyler and Wang, Yiqiao and Richmond, Dustin and Zhou, Peng and Eshraghian, Jason K},
  journal = {arXiv preprint arXiv:2406.02528},
  year    = {2024}
}
```

## Contributors

See [docs/CONTRIBUTORS](docs/CONTRIBUTORS).
