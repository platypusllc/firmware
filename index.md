---
layout: default
subsections: [, introduction, quickstart, troubleshooting, support, introduction]
---

# Homepage

This is your homepage!

To add a new section:

    bin/add_section.sh . section_name

To add a new post in a section:

    bin/add_new_post.sh ./path/to/section title

If you host your website on Github Pages, you might be interested in the base_dir option in _config.yaml: just set it to 

    base_dir: /project_name
