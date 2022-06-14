"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[197],{3905:function(e,t,n){n.d(t,{Zo:function(){return c},kt:function(){return m}});var a=n(67294);function i(e,t,n){return t in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function o(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var a=Object.getOwnPropertySymbols(e);t&&(a=a.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,a)}return n}function r(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?o(Object(n),!0).forEach((function(t){i(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):o(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function s(e,t){if(null==e)return{};var n,a,i=function(e,t){if(null==e)return{};var n,a,i={},o=Object.keys(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||(i[n]=e[n]);return i}(e,t);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(a=0;a<o.length;a++)n=o[a],t.indexOf(n)>=0||Object.prototype.propertyIsEnumerable.call(e,n)&&(i[n]=e[n])}return i}var l=a.createContext({}),p=function(e){var t=a.useContext(l),n=t;return e&&(n="function"==typeof e?e(t):r(r({},t),e)),n},c=function(e){var t=p(e.components);return a.createElement(l.Provider,{value:t},e.children)},d={inlineCode:"code",wrapper:function(e){var t=e.children;return a.createElement(a.Fragment,{},t)}},u=a.forwardRef((function(e,t){var n=e.components,i=e.mdxType,o=e.originalType,l=e.parentName,c=s(e,["components","mdxType","originalType","parentName"]),u=p(n),m=i,h=u["".concat(l,".").concat(m)]||u[m]||d[m]||o;return n?a.createElement(h,r(r({ref:t},c),{},{components:n})):a.createElement(h,r({ref:t},c))}));function m(e,t){var n=arguments,i=t&&t.mdxType;if("string"==typeof e||i){var o=n.length,r=new Array(o);r[0]=u;var s={};for(var l in t)hasOwnProperty.call(t,l)&&(s[l]=t[l]);s.originalType=e,s.mdxType="string"==typeof e?e:i,r[1]=s;for(var p=2;p<o;p++)r[p]=n[p];return a.createElement.apply(null,r)}return a.createElement.apply(null,n)}u.displayName="MDXCreateElement"},54366:function(e,t,n){n.r(t),n.d(t,{assets:function(){return c},contentTitle:function(){return l},default:function(){return m},frontMatter:function(){return s},metadata:function(){return p},toc:function(){return d}});var a=n(87462),i=n(63366),o=(n(67294),n(3905)),r=["components"],s={title:"VAST v2.0",description:"VAST v2.0 - Smarter Query Scheduling & Tunable Filters",authors:"dominiklohmann",date:new Date("2022-05-16T00:00:00.000Z"),tags:["release","compaction","performance","pcap"]},l=void 0,p={permalink:"/blog/vast-v2.0",source:"@site/blog/vast-v2.0/index.md",title:"VAST v2.0",description:"VAST v2.0 - Smarter Query Scheduling & Tunable Filters",date:"2022-05-16T00:00:00.000Z",formattedDate:"May 16, 2022",tags:[{label:"release",permalink:"/blog/tags/release"},{label:"compaction",permalink:"/blog/tags/compaction"},{label:"performance",permalink:"/blog/tags/performance"},{label:"pcap",permalink:"/blog/tags/pcap"}],readingTime:6.335,truncated:!0,authors:[{name:"Dominik Lohmann",title:"Engineering Manager",url:"https://github.com/dominiklohmann",email:"dominik@tenzir.com",imageURL:"https://github.com/dominiklohmann.png",key:"dominiklohmann"}],frontMatter:{title:"VAST v2.0",description:"VAST v2.0 - Smarter Query Scheduling & Tunable Filters",authors:"dominiklohmann",date:"2022-05-16T00:00:00.000Z",tags:["release","compaction","performance","pcap"]},nextItem:{title:"VAST v1.1.2",permalink:"/blog/vast-v1.1.2"}},c={authorsImageUrls:[void 0]},d=[{value:"Query Scheduling",id:"query-scheduling",level:2},{value:"Updates to Aging, Compaction, and the Disk Monitor",id:"updates-to-aging-compaction-and-the-disk-monitor",level:2},{value:"Fine-tuned Catalog Configuration",id:"fine-tuned-catalog-configuration",level:2},{value:"Configuring VAST with Environment Variables",id:"configuring-vast-with-environment-variables",level:2},{value:"VLAN Tag Extraction and Better Packet Decapsulation",id:"vlan-tag-extraction-and-better-packet-decapsulation",level:2},{value:"Breaking Changes",id:"breaking-changes",level:2},{value:"Changes for Developers",id:"changes-for-developers",level:2},{value:"Smaller Things",id:"smaller-things",level:2}],u={toc:d};function m(e){var t=e.components,s=(0,i.Z)(e,r);return(0,o.kt)("wrapper",(0,a.Z)({},u,s,{components:t,mdxType:"MDXLayout"}),(0,o.kt)("p",null,"Dear community, we are excited to announce ",(0,o.kt)("a",{parentName:"p",href:"https://github.com/tenzir/vast/releases/tag/v2.0.0"},"VAST v2.0"),",\nbringing faster execution of bulk-submitted queries, improved tunability of\nindex structures, and new configurability through environment variables."),(0,o.kt)("h2",{id:"query-scheduling"},"Query Scheduling"),(0,o.kt)("p",null,"VAST is now more intelligent in how it schedules queries."),(0,o.kt)("p",null,"When a query arrives at the VAST server, VAST first goes to the catalog which\nreturns a set of on-disk candidate partitions that the query may be applicable\nto. Previous versions of VAST simply iterated through the available queries as\nthey came in, loading partition by partition to extract events. Due to memory\nconstraints, VAST is only able to keep some partitions in memory, which causes\nfrequent loading and unloading of the same partitions for queries that access\nthe same data. Now, VAST loads partitions depending on how many queries they are\nrelevant for and evaluates all ongoing queries for one partition at a time."),(0,o.kt)("p",null,"Additionally, VAST now partitions the data for each schema separately, moving\naway from partitions that contain events of multiple schemas. This helps with\ncommon access patterns and speeds up queries restricted to a single schema."),(0,o.kt)("p",null,"The numbers speak for themselves:"),(0,o.kt)("p",null,(0,o.kt)("img",{alt:"Benchmarks",src:n(79919).Z+"#gh-light-mode-only",width:"3000",height:"2100"}),"\n",(0,o.kt)("img",{alt:"Benchmarks",src:n(41225).Z+"#gh-dark-mode-only",width:"3000",height:"2100"})),(0,o.kt)("h2",{id:"updates-to-aging-compaction-and-the-disk-monitor"},"Updates to Aging, Compaction, and the Disk Monitor"),(0,o.kt)("p",null,"VAST v1.0 deprecated the experimental aging feature. Given popular demand we've\ndecided to un-deprecate it and to actually implement it on top of the same\nbuilding blocks the new compaction mechanism uses, which means that it is now\nfully working and no longer considered experimental."),(0,o.kt)("p",null,'The compaction plugin is now able to apply general time-based compactions that\nare not restricted to a specific set of types. This makes it possible for\noperators to implement rules like "delete all data after 1 week", without having\nto list all possible data types that may occur.'),(0,o.kt)("p",null,"Some smaller interface changes improve the observability of the compactor for\noperators: The  ",(0,o.kt)("inlineCode",{parentName:"p"},"vast compaction status")," command prints the current compaction\nstatus, and the ",(0,o.kt)("inlineCode",{parentName:"p"},"vast compaction list")," command now lists all configured\ncompaction rules of the VAST node."),(0,o.kt)("p",null,"Additionally, we've improved overall stability and fault tolerance improvements\nsurrounding the disk monitor and compaction features."),(0,o.kt)("h2",{id:"fine-tuned-catalog-configuration"},"Fine-tuned Catalog Configuration"),(0,o.kt)("div",{className:"admonition admonition-note alert alert--secondary"},(0,o.kt)("div",{parentName:"div",className:"admonition-heading"},(0,o.kt)("h5",{parentName:"div"},(0,o.kt)("span",{parentName:"h5",className:"admonition-icon"},(0,o.kt)("svg",{parentName:"span",xmlns:"http://www.w3.org/2000/svg",width:"14",height:"16",viewBox:"0 0 14 16"},(0,o.kt)("path",{parentName:"svg",fillRule:"evenodd",d:"M6.3 5.69a.942.942 0 0 1-.28-.7c0-.28.09-.52.28-.7.19-.18.42-.28.7-.28.28 0 .52.09.7.28.18.19.28.42.28.7 0 .28-.09.52-.28.7a1 1 0 0 1-.7.3c-.28 0-.52-.11-.7-.3zM8 7.99c-.02-.25-.11-.48-.31-.69-.2-.19-.42-.3-.69-.31H6c-.27.02-.48.13-.69.31-.2.2-.3.44-.31.69h1v3c.02.27.11.5.31.69.2.2.42.31.69.31h1c.27 0 .48-.11.69-.31.2-.19.3-.42.31-.69H8V7.98v.01zM7 2.3c-3.14 0-5.7 2.54-5.7 5.68 0 3.14 2.56 5.7 5.7 5.7s5.7-2.55 5.7-5.7c0-3.15-2.56-5.69-5.7-5.69v.01zM7 .98c3.86 0 7 3.14 7 7s-3.14 7-7 7-7-3.12-7-7 3.14-7 7-7z"}))),"Advanced Users")),(0,o.kt)("div",{parentName:"div",className:"admonition-content"},(0,o.kt)("p",{parentName:"div"},"This section is for advanced users only."))),(0,o.kt)("p",null,"The catalog manages partition metadata and is responsible for deciding whether a\npartition qualifies for a certain query. It does so by maintaining sketch data\nstructures (e.g., Bloom filters, summary statistics) for each partition.\nSketches are highly space-efficient at the cost of being probabilistic and\nyielding false positives."),(0,o.kt)("p",null,"Due to this characteristic, sketches can grow sublinear: doubling the number of\nevents in a sketch does not lead to a doubling of the memory requirement.\nBecause the catalog must be traversed in full for a given query it needs to be\nmaintained in active memory to provide high responsiveness."),(0,o.kt)("p",null,"A false positive can have substantial impact on the query latency by\nmaterializing irrelevant partitions, which involves unnecessary I/O. Based on\nthe cost of I/O, this penalty may be substantial. Conversely, reducing the false\npositive rate increases the memory consumption, leading to a higher resident set\nsize and larger RAM requirements."),(0,o.kt)("p",null,"You can control this space-time trade-off in the configuration section\n",(0,o.kt)("inlineCode",{parentName:"p"},"vast.index")," by specifying index rules. Each rule corresponds to one sketch and\nconsists of the following components:"),(0,o.kt)("p",null,(0,o.kt)("inlineCode",{parentName:"p"},"targets"),": a list of extractors to describe the set of fields whose values to\nadd to the sketch. ",(0,o.kt)("inlineCode",{parentName:"p"},"fp-rate"),": an optional value to control the false-positive\nrate of the sketch."),(0,o.kt)("p",null,"VAST does not create field-level sketches unless a dedicated rule with a\nmatching target configuration exists. Here's an example:"),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-yaml"},"vast:\n  index:\n    rules:\n      - targets:\n          # field synopses: need to specify fully qualified field name\n          - suricata.http.http.url\n        fp-rate: 0.005\n      - targets:\n          - :addr\n        fp-rate: 0.1\n")),(0,o.kt)("p",null,"This configuration includes two rules (= two sketches), where the first rule\nincludes a field extractor and the second a type extractor. The first rule\napplies to a single field, ",(0,o.kt)("inlineCode",{parentName:"p"},"suricata.http.http.url"),", and has a false-positive\nrate of 0.5%. The second rule creates one sketch for all fields of type ",(0,o.kt)("inlineCode",{parentName:"p"},"addr"),"\nthat has a false-positive rate of 10%."),(0,o.kt)("h2",{id:"configuring-vast-with-environment-variables"},"Configuring VAST with Environment Variables"),(0,o.kt)("p",null,"VAST now offers an additional configuration path besides editing YAML\nconfiguration files and providing command line arguments: ",(0,o.kt)("em",{parentName:"p"},"setting environment\nvariables"),". This enables a convenient configuration experience when using\ncontainer runtimes, such as Docker, where the other two configuration paths have\na mediocre UX at best:"),(0,o.kt)("p",null,"The container entry point is limited to adding command line arguments, where not\nall options may be set. For Docker Compose and Kubernetes, it is often not\ntrivially possible to even add command line arguments."),(0,o.kt)("p",null,"Providing a manual configuration file is a heavy-weight action, because it\nrequires (1) generating a potentially templated configuration file, and (2)\nmounting that file into a location where VAST would read it."),(0,o.kt)("p",null,"An environment variable has the form ",(0,o.kt)("inlineCode",{parentName:"p"},"KEY=VALUE"),". VAST processes only\nenvironment variables having the form ",(0,o.kt)("inlineCode",{parentName:"p"},"VAST_{KEY}=VALUE"),". For example,\n",(0,o.kt)("inlineCode",{parentName:"p"},"VAST_ENDPOINT=1.2.3.4")," translates to the command line option\n",(0,o.kt)("inlineCode",{parentName:"p"},"--endpoint=1.2.3.4")," and YAML configuration ",(0,o.kt)("inlineCode",{parentName:"p"},"vast.endpoint: 1.2.3.4"),"."),(0,o.kt)("p",null,"Regarding precedence, environment variables override configuration file\nsettings, and command line arguments override environment variables. Please\nconsult the ",(0,o.kt)("a",{parentName:"p",href:"/docs/setup-vast/configure#environment-variables"},"documentation"),"\nfor a more detailed explanation of how to specify keys and values."),(0,o.kt)("h2",{id:"vlan-tag-extraction-and-better-packet-decapsulation"},"VLAN Tag Extraction and Better Packet Decapsulation"),(0,o.kt)("p",null,"VAST now extracts ",(0,o.kt)("a",{parentName:"p",href:"https://en.wikipedia.org/wiki/IEEE_802.1Q"},"802.1Q VLAN tags"),"\nfrom packets, making it possible to filter packets based on VLAN ID. The packet\nschema includes a new nested record ",(0,o.kt)("inlineCode",{parentName:"p"},"vlan")," with two fields: ",(0,o.kt)("inlineCode",{parentName:"p"},"outer")," and ",(0,o.kt)("inlineCode",{parentName:"p"},"inner"),"\nto represent the respective VLAN ID. For example, you can generate PCAP traces\nof packets based on VLAN IDs as follows:"),(0,o.kt)("pre",null,(0,o.kt)("code",{parentName:"pre",className:"language-bash"},"vast export pcap 'vlan.outer > 0 || vlan.inner in [1, 2, 3]' | tcpdump -r - -nl\n")),(0,o.kt)("p",null,"VLAN tags occur in many variations, and VAST extracts them in case of\nsingle-tagging and  ",(0,o.kt)("a",{parentName:"p",href:"https://en.wikipedia.org/wiki/IEEE_802.1ad"},"QinQ\ndouble-tagging"),". Consult the ",(0,o.kt)("a",{parentName:"p",href:"/docs/use-vast/ingest#pcap"},"PCAP\ndocumentation")," for details on this feature."),(0,o.kt)("p",null,"Internally, the packet decapsulation logic has been rewritten to follow a\nlayered approach: frames, packets, and segments are the building blocks. The\nplan is to reuse this architecture when switching to kernel-bypass packet\nacquisition using DPDK. If you would like to see more work on the front of\nhigh-performance packet recording, please reach out."),(0,o.kt)("h2",{id:"breaking-changes"},"Breaking Changes"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"--verbosity")," command-line option is now called ",(0,o.kt)("inlineCode",{parentName:"p"},"--console-verbosity"),". The\nshorthand options ",(0,o.kt)("inlineCode",{parentName:"p"},"-v"),", ",(0,o.kt)("inlineCode",{parentName:"p"},"-vv"),", ",(0,o.kt)("inlineCode",{parentName:"p"},"-vvv"),", ",(0,o.kt)("inlineCode",{parentName:"p"},"-q"),", ",(0,o.kt)("inlineCode",{parentName:"p"},"-qq"),", and  ",(0,o.kt)("inlineCode",{parentName:"p"},"-qqq"),"  are unchanged.\nThis aligns the command-line option with the configuration option\n",(0,o.kt)("inlineCode",{parentName:"p"},"vast.console-verbosity"),", and disambiguates from the ",(0,o.kt)("inlineCode",{parentName:"p"},"vast.file-verbosity"),"\noption."),(0,o.kt)("p",null,"The ",(0,o.kt)("em",{parentName:"p"},"Meta Index")," is now called the ",(0,o.kt)("em",{parentName:"p"},"Catalog"),". This affects multiple status and\nmetrics keys. We plan to extend the functionality of the Catalog in a future\nrelease, turning it into a more powerful first instance for lookups."),(0,o.kt)("p",null,"Transform steps that add or modify columns now add or modify the columns\nin-place rather than at the end, preserving the nesting structure of the\noriginal data."),(0,o.kt)("h2",{id:"changes-for-developers"},"Changes for Developers"),(0,o.kt)("p",null,"The ",(0,o.kt)("inlineCode",{parentName:"p"},"vast get")," command no longer exists. The command allowed for retrieving\nevents by their internal unique ID, which we are looking to remove entirely in\nthe future."),(0,o.kt)("p",null,"Changes to the internal data representation of VAST require all transform step\nplugins to be updated. The output format of the vast export arrow command\nchanged for the address, subnet, pattern, and enumeration types, which are now\nmodeled as ",(0,o.kt)("a",{parentName:"p",href:"https://arrow.apache.org/docs/format/Columnar.html#extension-types"},"Arrow Extension\nTypes"),". The\nrecord type is no longer flattened. The mapping of VAST types to Apache Arrow\ndata types  is now considered stable."),(0,o.kt)("h2",{id:"smaller-things"},"Smaller Things"),(0,o.kt)("ul",null,(0,o.kt)("li",{parentName:"ul"},"VAST client commands now start much faster and use less memory."),(0,o.kt)("li",{parentName:"ul"},"The ",(0,o.kt)("inlineCode",{parentName:"li"},"vast count --estimate '<query>'")," feature no longer unnecessarily causes\nstores to load from disk, resulting in major speedups for larger databases and\nbroad queries."),(0,o.kt)("li",{parentName:"ul"},"The ",(0,o.kt)("a",{parentName:"li",href:"https://github.com/tenzir/vast"},"tenzir/vast")," repository now contains\nexperimental Terraform scripts for deploying VAST to AWS Fargate and Lambda.")))}m.isMDXComponent=!0},41225:function(e,t,n){t.Z=n.p+"assets/images/scheduler-dark-494294caab31783eaf51c54055597463.png"},79919:function(e,t,n){t.Z=n.p+"assets/images/scheduler-light-a41d3be0b89de1169e9d96fd5d927da1.png"}}]);