# R4W Presentations

Slide decks for presenting R4W to various audiences.

## Available Presentations

| File | Audience | Duration | Focus |
|------|----------|----------|-------|
| [01_platform_overview.md](01_platform_overview.md) | Technical leadership | 20 min | Vision, capabilities, roadmap |
| [02_technical_deep_dive.md](02_technical_deep_dive.md) | Engineers | 45 min | Architecture, code, demos |
| [03_workshop_intro.md](03_workshop_intro.md) | Workshop attendees | 15 min | Setup, goals, exercises |
| [04_why_rust_for_sdr.md](04_why_rust_for_sdr.md) | C/C++ developers | 30 min | Migration case, safety, perf |
| [05_r4w_demo.md](05_r4w_demo.md) | General audience | 30 min | Screenshots, features tour |
| [06_fpga_acceleration.md](06_fpga_acceleration.md) | FPGA engineers | 25 min | IP cores, Zynq, Lattice |
| [07_security_isolation.md](07_security_isolation.md) | Security engineers | 25 min | 8 isolation levels, sandboxing |
| [08_waveform_development.md](08_waveform_development.md) | Waveform developers | 30 min | Trait system, implementation |
| [09_realtime_systems.md](09_realtime_systems.md) | Systems engineers | 25 min | Scheduling, FHSS, TDMA |
| [10_workshop_series.md](10_workshop_series.md) | Training coordinators | 20 min | 11 workshops overview |

## Building Slides

Use the Makefile targets to build reveal.js HTML slideshows:

```bash
# Build all slideshows
make slides

# Build a single slideshow
make slide NAME=05_r4w_demo

# Open demo slideshow in browser
make slides-demo

# List available presentations
make slides-list

# Clean generated slides
make slides-clean
```

Generated HTML slides are placed in `docs/slides/`.

## Presentation Format

Presentations are in Markdown for:
- Version control with Git
- Easy editing
- Export via pandoc/reveal.js
- Web viewing via GitHub

## Screenshots

Screenshots of r4w-explorer are stored in `images/screenshots/`. Use them in presentations with:

```markdown
![Spectrum View](images/screenshots/spectrum_view.png)
```

Note: When slides are built, images are copied to `docs/slides/images/`.

## Customization

Edit presentations as needed for your audience. Key sections to customize:
- Company/organization name
- Specific use cases relevant to audience
- Hardware available for demos
- Timeline/roadmap based on priorities

## Styling

Presentations use reveal.js with the `night` theme. Custom CSS can be added in the YAML header:

```yaml
header-includes: |
  <style>
    .reveal pre { margin: 0 auto; width: fit-content; }
  </style>
```
