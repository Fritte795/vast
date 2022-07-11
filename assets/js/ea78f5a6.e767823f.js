"use strict";(self.webpackChunkdocs=self.webpackChunkdocs||[]).push([[9270],{3905:(e,r,t)=>{t.d(r,{Zo:()=>u,kt:()=>m});var n=t(67294);function a(e,r,t){return r in e?Object.defineProperty(e,r,{value:t,enumerable:!0,configurable:!0,writable:!0}):e[r]=t,e}function o(e,r){var t=Object.keys(e);if(Object.getOwnPropertySymbols){var n=Object.getOwnPropertySymbols(e);r&&(n=n.filter((function(r){return Object.getOwnPropertyDescriptor(e,r).enumerable}))),t.push.apply(t,n)}return t}function s(e){for(var r=1;r<arguments.length;r++){var t=null!=arguments[r]?arguments[r]:{};r%2?o(Object(t),!0).forEach((function(r){a(e,r,t[r])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(t)):o(Object(t)).forEach((function(r){Object.defineProperty(e,r,Object.getOwnPropertyDescriptor(t,r))}))}return e}function i(e,r){if(null==e)return{};var t,n,a=function(e,r){if(null==e)return{};var t,n,a={},o=Object.keys(e);for(n=0;n<o.length;n++)t=o[n],r.indexOf(t)>=0||(a[t]=e[t]);return a}(e,r);if(Object.getOwnPropertySymbols){var o=Object.getOwnPropertySymbols(e);for(n=0;n<o.length;n++)t=o[n],r.indexOf(t)>=0||Object.prototype.propertyIsEnumerable.call(e,t)&&(a[t]=e[t])}return a}var p=n.createContext({}),l=function(e){var r=n.useContext(p),t=r;return e&&(t="function"==typeof e?e(r):s(s({},r),e)),t},u=function(e){var r=l(e.components);return n.createElement(p.Provider,{value:r},e.children)},d={inlineCode:"code",wrapper:function(e){var r=e.children;return n.createElement(n.Fragment,{},r)}},c=n.forwardRef((function(e,r){var t=e.components,a=e.mdxType,o=e.originalType,p=e.parentName,u=i(e,["components","mdxType","originalType","parentName"]),c=l(t),m=a,f=c["".concat(p,".").concat(m)]||c[m]||d[m]||o;return t?n.createElement(f,s(s({ref:r},u),{},{components:t})):n.createElement(f,s({ref:r},u))}));function m(e,r){var t=arguments,a=r&&r.mdxType;if("string"==typeof e||a){var o=t.length,s=new Array(o);s[0]=c;var i={};for(var p in r)hasOwnProperty.call(r,p)&&(i[p]=r[p]);i.originalType=e,i.mdxType="string"==typeof e?e:a,s[1]=i;for(var l=2;l<o;l++)s[l]=t[l];return n.createElement.apply(null,s)}return n.createElement.apply(null,t)}c.displayName="MDXCreateElement"},5534:(e,r,t)=>{t.r(r),t.d(r,{assets:()=>p,contentTitle:()=>s,default:()=>d,frontMatter:()=>o,metadata:()=>i,toc:()=>l});var n=t(87462),a=(t(67294),t(3905));const o={},s="drop",i={unversionedId:"understand-vast/query-language/operators/drop",id:"understand-vast/query-language/operators/drop",title:"drop",description:"Drops individual fields having the configured extractors from the input or",source:"@site/docs/understand-vast/query-language/operators/drop.md",sourceDirName:"understand-vast/query-language/operators",slug:"/understand-vast/query-language/operators/drop",permalink:"/docs/understand-vast/query-language/operators/drop",draft:!1,editUrl:"https://github.com/tenzir/vast/tree/master/web/docs/understand-vast/query-language/operators/drop.md",tags:[],version:"current",frontMatter:{},sidebar:"docsSidebar",previous:{title:"Operators",permalink:"/docs/understand-vast/query-language/operators/"},next:{title:"hash",permalink:"/docs/understand-vast/query-language/operators/hash"}},p={},l=[{value:"Parameters",id:"parameters",level:2},{value:"Example",id:"example",level:2}],u={toc:l};function d(e){let{components:r,...t}=e;return(0,a.kt)("wrapper",(0,n.Z)({},u,t,{components:r,mdxType:"MDXLayout"}),(0,a.kt)("h1",{id:"drop"},"drop"),(0,a.kt)("p",null,"Drops individual fields having the configured extractors from the input or\nentire schemas."),(0,a.kt)("p",null,"The ",(0,a.kt)("inlineCode",{parentName:"p"},"drop")," operator is the dual to ",(0,a.kt)("a",{parentName:"p",href:"put"},(0,a.kt)("inlineCode",{parentName:"a"},"put")),", which selects a given set of\nfields from the output."),(0,a.kt)("h2",{id:"parameters"},"Parameters"),(0,a.kt)("ul",null,(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"fields: [string]"),": The extractors of fields to drop."),(0,a.kt)("li",{parentName:"ul"},(0,a.kt)("inlineCode",{parentName:"li"},"schemas: [string]"),": The names of schemas to drop.")),(0,a.kt)("h2",{id:"example"},"Example"),(0,a.kt)("pre",null,(0,a.kt)("code",{parentName:"pre",className:"language-yaml"},"drop:\n  fields:\n    # Remove the source_ip and dest_ip columns if they exist\n    - source_ip\n    - dest_ip\n  schemas:\n    # Drop all suricata.dns events in their entirety\n    - suricata.dns\n")))}d.isMDXComponent=!0}}]);