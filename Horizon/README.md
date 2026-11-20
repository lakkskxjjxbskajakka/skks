GOAL:
پروژه: طراحی و توسعه یک موتور بازی‌سازی سه‌بعدی فوق‌بهینه با زبان C

۱. مأموریت اصلی پروژه




یک موتور بازی‌سازی سه‌بعدی (3D Game Engine) حرفه‌ای، ماژولار، چندسکویی و فوق‌العاده بهینه طراحی و پیاده‌سازی کن.




زبان اصلی توسعه باید C17 یا C23 باشد.




هدف اصلی این پروژه دستیابی به این ترکیب است:




- حداکثر کارایی پردازنده (CPU).
- حداکثر بهره‌وری پردازنده گرافیکی (GPU).
- حداقل مصرف RAM و VRAM.
- حداقل مصرف پهنای باند حافظه.
- بیشترین نرخ فریم ممکن.
- بالاترین کیفیت بصری قابل‌دستیابی با توجه به محدودیت سخت‌افزار.
- واقع‌گرایانه‌ترین گرافیک ممکن با کمترین هزینه پردازشی.
- مصرف انرژی و تولید حرارت کنترل‌شده.
- پشتیبانی Native از ویندوز، لینوکس و اندروید.




فلسفه اصلی پروژه:




Maximum Visual Fidelity per Millisecond, Maximum Performance per Byte, Minimum Overhead.




این موتور باید بتواند روی سخت‌افزارهای ضعیف نیز عملکرد مناسبی داشته باشد و در سخت‌افزارهای قدرتمند از تمام ظرفیت موجود استفاده کند.




منظور از سخت‌افزار ضعیف، دستگاهی است که حداقل یکی از منابع CPU، GPU، RAM یا پهنای باند حافظه آن محدود باشد، اما درایور گرافیکی سازگار و سالم داشته باشد.




هیچ ادعای غیرواقعی درباره دستیابی هم‌زمان به حداکثر کیفیت و FPS روی سخت‌افزارهای محدود پذیرفته نیست. وظیفه موتور، یافتن بهترین تعادل قابل‌اندازه‌گیری میان این دو است.




---




۲. زبان برنامه‌نویسی و اصول مهندسی




زبان اصلی: C17 یا C23.




تمام بخش‌های حیاتی Runtime باید با C پیاده‌سازی شوند.




اصول اجباری:




- استفاده از Native Compilation.
- عدم نیاز به Garbage Collector.
- عدم استفاده از Runtime سنگین.
- مدیریت دستی و دقیق حافظه.
- حداقل وابستگی خارجی.
- عدم استفاده از Abstractionهای غیرضروری.
- طراحی Cache-Friendly.
- استفاده از SIMD در نقاطی که واقعاً سودمند است.
- کنترل دقیق Threadها و حافظه.
- طراحی APIهای ساده و کم‌هزینه.
- جلوگیری از Dynamic Allocation غیرضروری در مسیر Frame.
- جلوگیری از Pointer Chasing غیرضروری.




از Data-Oriented Design در سیستم‌هایی که به عملکرد بالا نیاز دارند استفاده کن.




از Object-Oriented Design صرفاً در مواردی استفاده کن که مزیت معماری آن از هزینه احتمالی بیشتر باشد.




هیچ بهینه‌سازی نباید بدون Benchmark و Profiling معتبر پذیرفته شود.




---




۳. پشتیبانی چندسکویی (Cross-Platform)




پشتیبانی از سه سیستم‌عامل زیر از الزامات اصلی معماری است:




Windows




- پشتیبانی Native از Windows 10 و Windows 11.
- خروجی اجرایی Native با فرمت EXE.
- پشتیبانی از Vulkan.
- پشتیبانی از Direct3D 12 در صورت توجیه معماری.
- پشتیبانی از OpenGL.
- مدیریت Native پنجره و ورودی.
- مدیریت فایل، حافظه و Threadها.
- پشتیبانی از سیستم‌های 64 بیتی.




Linux




- پشتیبانی از توزیع‌های رایج Linux.
- خروجی Native ELF.
- پشتیبانی از X11.
- پشتیبانی از Wayland.
- پشتیبانی از Vulkan.
- پشتیبانی از OpenGL.
- مدیریت Native پنجره و ورودی.
- سازگاری با سیستم‌های Linux مبتنی بر سخت‌افزار ضعیف.
- عدم وابستگی اجباری به یک Desktop Environment خاص.




لایه Platform باید از Window System مستقل باشد و امکان استفاده از X11 و Wayland را فراهم کند.




Android




- پشتیبانی از Android با استفاده از Android NDK.
- خروجی Native قابل‌استفاده در APK و AAB.
- استفاده از C در بخش Runtime.
- پشتیبانی از Vulkan.
- پشتیبانی از OpenGL ES 3.x.
- پشتیبانی از ARM64.
- امکان پشتیبانی از ARMv7 در صورت توجیه فنی.
- مدیریت Touch Input.
- مدیریت Lifecycle برنامه.
- مدیریت محدودیت RAM.
- مدیریت مصرف باتری و حرارت.
- پشتیبانی از Asset Streaming.
- مدیریت وقفه‌ها و بازگشت برنامه به حالت فعال.




از Android GameActivity برای ارتباط مناسب با Lifecycle و ورودی اندروید استفاده کن.




اصل مهم




منطق اصلی موتور نباید برای هر سیستم‌عامل جداگانه نوشته شود.




معماری باید به شکل زیر باشد:




Engine Core




- Windows Platform Backend
- Linux Platform Backend
- Android Platform Backend




سیستم‌های Core، ECS، Physics، Scene، Asset Management و منطق Rendering باید تا حد امکان مستقل از سیستم‌عامل باشند.




---




۴. معماری Graphics API




یک لایه Rendering Hardware Interface یا RHI اختصاصی طراحی کن.




Backendهای موردنیاز:




1. Vulkan
2. OpenGL ES 3.x
3. OpenGL 4.x
4. Direct3D 12 به‌عنوان Backend توسعه آینده
5. Metal به‌عنوان Backend توسعه آینده




Vulkan باید Backend اصلی برای سخت‌افزارهای مدرن باشد.




OpenGL ES باید مسیر سازگاری برای دستگاه‌های اندرویدی فاقد پشتیبانی مناسب Vulkan باشد.


OpenGL نیز برای سازگاری با سخت‌افزارهای دسکتاپ در نظر گرفته شود.




RHI باید:




- کمترین Overhead ممکن را داشته باشد.
- از Function Tableهای سبک C و Handleهای مناسب استفاده کند.
- از تخصیص حافظه غیرضروری جلوگیری کند.
- قابلیت‌های واقعی GPU را شناسایی کند.
- Resource Management مناسبی داشته باشد.
- مدیریت Synchronization را بهینه انجام دهد.
- از Pipeline و Shader Cache پشتیبانی کند.
- قابلیت انتخاب Backend را در زمان اجرا فراهم کند.




هیچ Backend نباید صرفاً به دلیل جدیدتر بودن سریع‌تر فرض شود.




برای هر دستگاه، بهترین Backend باید با توجه به قابلیت‌ها و نتایج اندازه‌گیری انتخاب شود.




---




۵. معماری Rendering




یک Renderer چندمسیره طراحی کن.




روش‌های زیر را بررسی، پیاده‌سازی و Benchmark کن:




- Forward Rendering
- Forward+
- Deferred Rendering
- Tiled Rendering
- Clustered Forward Rendering
- GPU-Driven Rendering
- Hybrid Rendering




انتخاب نهایی باید بر اساس نوع GPU، پهنای باند حافظه، ظرفیت VRAM، قابلیت‌های API و هزینه واقعی هر روش باشد.




برای سخت‌افزارهای ضعیف، مسیرهای کم‌هزینه و کم‌مصرف در نظر بگیر.




برای سخت‌افزارهای قدرتمند، قابلیت‌های گرافیکی پیشرفته‌تر فعال شوند.




Renderer نباید به یک مسیر ثابت محدود باشد.




Render Graph




یک Render Graph سبک و بهینه طراحی کن که بتواند:




- وابستگی Passها را مدیریت کند.
- ترتیب اجرای عملیات را تعیین کند.
- Resource Lifetime را تحلیل کند.
- امکان Resource Aliasing را فراهم کند.
- Synchronization غیرضروری را کاهش دهد.
- از Memory Reuse استفاده کند.




پیاده‌سازی Render Graph نباید باعث ایجاد هزینه CPU غیرضروری در هر Frame شود.




---




۶. واقع‌گرایی گرافیکی (Photorealistic Rendering)




کیفیت بصری یکی از مهم‌ترین اهداف پروژه است.




از روش‌های مدرن Computer Graphics و Physically Based Rendering استفاده کن.




قابلیت‌های موردنیاز:




Materials




- Physically Based Rendering
- Metallic-Roughness Workflow
- Normal Mapping
- Height Mapping
- Parallax Occlusion Mapping در صورت توجیه
- Material Layers
- Texture Compression
- Material Instancing




Lighting




- Physically Based Lighting
- Image-Based Lighting
- HDR
- Environment Maps
- Reflection Probes
- Baked Lighting
- Dynamic Lighting
- Hybrid Global Illumination




Shadows




- Shadow Mapping
- Cascaded Shadow Maps
- PCF
- VSM در صورت توجیه
- Contact Shadows
- Shadow LOD
- Adaptive Shadow Resolution




Ambient Lighting




- SSAO
- GTAO
- HBAO در صورت توجیه
- Baked Ambient Occlusion
- ترکیب روش‌های Screen Space و Baked




Reflections




- Screen Space Reflections
- Reflection Probes
- Planar Reflections
- روش‌های مبتنی بر Ray Tracing در سخت‌افزارهای پشتیبانی‌شده




Post Processing




- Tone Mapping
- Bloom
- Color Grading
- Depth of Field
- Motion Blur
- Anti-Aliasing
- Temporal Anti-Aliasing
- Sharpening
- Exposure Control




Environment




- Realistic Sky
- Atmospheric Scattering
- Fog
- Volumetric Fog در صورت توجیه
- Clouds
- Water Rendering
- Terrain Rendering
- Vegetation Rendering




تمام این قابلیت‌ها نباید به‌صورت اجباری فعال باشند.




هر سیستم باید قابلیت تنظیم مستقل داشته باشد.




کیفیت بصری باید با توجه به توان واقعی سخت‌افزار تنظیم شود.




از تکنیک‌های Baked، Hybrid و Approximation برای کاهش هزینه پردازشی استفاده کن، مشروط بر اینکه کیفیت بصری قابل‌قبولی حفظ شود.




---




۷. بهینه‌سازی شدید CPU




یک معماری اختصاصی برای استفاده بهینه از CPU طراحی کن.




موارد الزامی:




- Data-Oriented Design
- Cache-Friendly Memory Layout
- Structure of Arrays
- SIMD
- Multithreaded Job System
- Work Stealing در صورت توجیه
- Lock-Free Structures در نقاط مناسب
- کاهش Context Switching
- کاهش Lock Contention
- Batch Processing
- Efficient Scene Traversal
- Spatial Partitioning
- Hierarchical Data Structures




Job System باید بتواند عملیات مستقل را بین هسته‌های CPU تقسیم کند.




از ایجاد Threadهای بیش‌ازحد خودداری کن.




برای پردازنده‌های ضعیف و کم‌هسته نیز Scheduler مناسبی داشته باش.




---




۸. بهینه‌سازی GPU




تمام سیستم‌های Rendering باید با هدف کاهش هزینه GPU طراحی شوند.




قابلیت‌های موردنیاز:


- GPU Instancing
- Indirect Drawing
- Frustum Culling
- Occlusion Culling
- Hierarchical Z-Buffer
- Mesh LOD
- HLOD
- Texture Mipmapping
- Texture Streaming
- Shader Optimization
- Draw Call Batching
- Overdraw Reduction
- Efficient Shadow Rendering
- GPU Resource Reuse




در GPUهای Tile-Based موبایل، معماری Rendering باید با درنظرگرفتن هزینه Load/Store و پهنای باند حافظه طراحی شود.




از Readback غیرضروری GPU به CPU اجتناب کن.




از Synchronizationهای غیرضروری جلوگیری کن.




---




۹. سیستم حافظه اختصاصی




یک Memory Management System مستقل طراحی کن.




Allocatorهای موردنیاز:




- Arena Allocator
- Linear Allocator
- Pool Allocator
- Stack Allocator
- Free List Allocator
- GPU Memory Allocator
- Asset Memory Manager




اهداف:




- کاهش Fragmentation.
- کاهش Allocationهای کوچک و پرتعداد.
- کاهش حافظه مصرفی.
- کنترل Lifetime منابع.
- استفاده مجدد از حافظه.
- جلوگیری از Memory Leak.
- مدیریت مناسب RAM و VRAM.




تمام سیستم‌های حافظه باید قابلیت Profiling داشته باشند.




---




۱۰. Adaptive Performance و Quality System




یک سیستم هوشمند برای تنظیم کیفیت گرافیکی پیاده‌سازی کن.




این سیستم باید اطلاعات زیر را بررسی کند:




- GPU Frame Time
- CPU Frame Time
- Average FPS
- Frame Time Variance
- RAM Usage
- VRAM Usage
- GPU Capabilities
- Available Memory
- Thermal Status در پلتفرم‌های پشتیبانی‌شده
- Power Constraints




سپس در صورت نیاز موارد زیر را تنظیم کند:




- Resolution
- Render Scale
- Shadow Resolution
- Shadow Distance
- LOD Distance
- Texture Quality
- Reflection Quality
- Ambient Occlusion
- Anti-Aliasing
- Lighting Complexity
- Particle Density
- Draw Distance




هدف صرفاً رسیدن به FPS مشخص نیست؛ هدف دستیابی به بهترین کیفیت بصری پایدار در محدودیت سخت‌افزار است.




از تغییرات ناگهانی و آزاردهنده کیفیت جلوگیری کن.




---




۱۱. ECS و Scene Management




یک ECS اختصاصی یا پیاده‌سازی بهینه و مناسب انتخاب کن.




ویژگی‌ها:




- Entity Management
- Component Storage
- Archetype Storage در صورت توجیه
- Efficient Querying
- Transform Hierarchy
- Scene Graph
- Serialization
- Scene Loading
- Scene Streaming




از طراحی پیچیده و غیرضروری خودداری کن.




ECS نباید به یک محدودیت معماری تبدیل شود.




برای هر سیستم، ساختار داده‌ای مناسب را انتخاب کن.




---




۱۲. Physics و Animation




یک سیستم Physics ماژولار طراحی کن.




ویژگی‌ها:




- Collision Detection
- Broad Phase
- Narrow Phase
- Rigid Body Simulation
- Spatial Acceleration Structures
- Collision Layers
- Ray Casting
- Basic Vehicle Physics




برای Animation:




- Skeletal Animation
- Animation Blending
- Animation State Machine
- GPU Skinning در صورت توجیه
- Animation Compression




در صورت استفاده از کتابخانه خارجی، حجم، مجوز، عملکرد و وابستگی‌های آن را بررسی کن.




---




۱۳. Asset Pipeline




یک سیستم مستقل برای مدیریت دارایی‌های بازی طراحی کن.




فرمت‌ها و قابلیت‌ها:




- Mesh Import
- Texture Import
- Material Import
- Animation Import
- Audio Import
- Asset Compression
- Asset Bundling
- Resource Streaming
- Asynchronous Loading
- Asset Caching




از فرمت‌های استاندارد و مناسب مانند glTF 2.0 استفاده کن.




امکان تبدیل Assetها به فرمت Binary اختصاصی و بهینه موتور وجود داشته باشد.




سیستم Asset باید قابلیت کار روی ویندوز، لینوکس و اندروید را داشته باشد.




---




۱۴. ابزار توسعه و Editor




Editor باید یک ابزار مستقل از Runtime باشد.




قابلیت‌های موردنیاز در نسخه‌های بعدی:




- Scene Editor
- Entity Inspector
- Material Editor
- Texture Viewer
- Mesh Viewer
- Asset Browser
- Performance Profiler
- Memory Profiler
- Shader Debugging
- Project Management




Editor می‌تواند از فناوری متفاوتی نسبت به Runtime استفاده کند، اما هسته موتور باید همچنان C باقی بماند.




Editor نباید باعث سنگین‌شدن خروجی بازی شود.




---




۱۵. سیستم Benchmark و Profiling




هیچ ادعای عملکردی بدون آزمایش معتبر پذیرفته نیست.




یک Benchmark Suite اختصاصی بساز.




معیارها:




- Average FPS
- 1% Low FPS
- 0.1% Low FPS
- Frame Time
- CPU Usage
- GPU Usage
- RAM Consumption
- VRAM Consumption
- Draw Calls
- Triangle Count
- Shader Cost
- Loading Time
- Energy Consumption در صورت امکان اندازه‌گیری معتبر




صحنه‌های Benchmark:


1. محیط طبیعی بزرگ با پوشش گیاهی.
2. جنگل متراکم.
3. شهر بزرگ با ساختمان‌های متعدد.
4. صحنه دارای نورپردازی پیچیده.
5. صحنه دارای Shadowهای متعدد.
6. تعداد زیاد اشیای متحرک.
7. صحنه دارای متریال‌های PBR.
8. صحنه مخصوص GPUهای موبایل.




تمام Benchmarkها باید با تنظیمات یکسان و شرایط مستند انجام شوند.




نتایج ساختگی یا تخمینی را به‌عنوان نتیجه واقعی ارائه نکن.




---




۱۶. Build System و خروجی نهایی




سیستم Build باید قابلیت تولید خروجی مستقل برای هر پلتفرم را داشته باشد.




Windows




- EXE
- DLL در صورت نیاز
- Release و Debug
- x86_64
- امکان توسعه ARM64 در آینده




Linux




- ELF
- Shared Library در صورت نیاز
- Release و Debug
- x86_64
- ARM64




Android




- APK
- AAB
- ARM64
- امکان ARMv7 در صورت نیاز
- پشتیبانی از Vulkan و OpenGL ES




از CMake برای مدیریت Build چندسکویی استفاده کن، مگر اینکه دلیل فنی مستندی برای انتخاب سیستم دیگری وجود داشته باشد.




Compilerهای GCC، Clang و MSVC را در فرآیند توسعه و تست در نظر بگیر.




تنظیمات Compiler باید برای هر پلتفرم به‌صورت مستقل و اصولی انتخاب شوند.




---




۱۷. ساختار پیشنهادی پروژه




Engine/
|
|-- Core/
|   |-- Memory/
|   |-- Math/
|   |-- Containers/
|   |-- Logging/
|   |-- Jobs/
|   |-- Profiling/
|   |-- Platform/
|
|-- Renderer/
|   |-- RHI/
|   |-- Vulkan/
|   |-- OpenGL/
|   |-- RenderGraph/
|   |-- Lighting/
|   |-- Materials/
|   |-- Shadows/
|   |-- Culling/
|   |-- PostProcessing/
|   |-- Shaders/
|
|-- ECS/
|-- Scene/
|-- Physics/
|-- Animation/
|-- Audio/
|-- Assets/
|-- Serialization/
|
|-- Platforms/
|   |-- Windows/
|   |-- Linux/
|   |-- Android/
|
|-- Editor/
|-- Tools/
|-- Tests/
|-- Benchmarks/
|-- Examples/
|
|-- CMakeLists.txt
|-- README.md
|-- LICENSE




این ساختار اولیه است و باید با پیشرفت پروژه و نیازهای واقعی معماری اصلاح شود.




---




۱۸. مراحل توسعه




پروژه را به مراحل مستقل تقسیم کن.




مرحله اول: Core و Memory Management.




مرحله دوم: Platform Abstraction و Window System.




مرحله سوم: Vulkan Initialization و Renderer پایه.




مرحله چهارم: Mesh، Camera، Texture و Material.




مرحله پنجم: OpenGL و OpenGL ES Backend.




مرحله ششم: Profiling و Benchmarking.




مرحله هفتم: Culling، LOD، Instancing و Job System.




مرحله هشتم: PBR و سیستم‌های گرافیکی واقع‌گرایانه.




مرحله نهم: ECS، Scene، Physics و Animation.




مرحله دهم: Android Backend و خروجی APK.




مرحله یازدهم: تکمیل Windows و Linux.




مرحله دوازدهم: Asset Pipeline و Editor.




مرحله سیزدهم: Optimization، تست نهایی و آماده‌سازی Production.




ترتیب مراحل در صورت وجود دلیل مهندسی معتبر قابل تغییر است.




---




۱۹. قوانین اجباری توسعه




۱. هیچ‌وقت قابلیت‌های متعدد را به‌صورت نمایشی و ناقص پیاده‌سازی نکن.




۲. کد باید واقعی، قابل‌کامپایل و قابل‌آزمایش باشد.




۳. از Placeholder به‌جای پیاده‌سازی واقعی استفاده نکن، مگر اینکه صریحاً مشخص شده باشد.




۴. هر ماژول باید مسئولیت مشخصی داشته باشد.




۵. قبل از طراحی هر سیستم، گزینه‌های معماری و هزینه‌های آن‌ها را بررسی کن.




۶. قبل از بهینه‌سازی، Benchmark بگیر.




۷. بعد از بهینه‌سازی، دوباره Benchmark بگیر.




۸. نتایج را مستند کن.




۹. Memory Leak، Undefined Behavior و Data Race قابل‌قبول نیستند.




۱۰. قابلیت‌های پلتفرم‌های مختلف را واقعاً تست کن.




۱۱. اگر امکان تست یک پلتفرم وجود ندارد، این محدودیت را صریح اعلام کن.




۱۲. از وابستگی‌های غیرضروری پرهیز کن.




۱۳. از قابلیت‌های سخت‌افزاری اختیاری به‌صورت Capability-Based استفاده کن.




۱۴. سازگاری و کارایی را با هم در نظر بگیر.




۱۵. از طراحی بیش‌ازحد پیچیده و Premature Optimization جلوگیری کن.




۱۶. هر مرحله باید خروجی قابل‌اجرا و قابل‌تست داشته باشد.




۱۷. مستندات فنی، APIها و دستورالعمل Build باید هم‌زمان با توسعه نوشته شوند.




۱۸. هیچ‌گاه ادعا نکن که موتور Production-Ready است مگر اینکه معیارهای مشخص تست و کیفیت را گذرانده باشد.




---




۲۰. مأموریت نهایی




هدف، ساخت یک موتور سه‌بعدی Native با زبان C است که بتواند روی ویندوز، لینوکس و اندروید اجرا شود.




این موتور باید در سخت‌افزارهای محدود، با استفاده از معماری هوشمند، الگوریتم‌های کارآمد و مدیریت دقیق منابع، بالاترین کیفیت بصری قابل‌دستیابی را ارائه دهد.




در سخت‌افزارهای قدرتمند نیز باید بتواند از قابلیت‌های مدرن CPU و GPU به‌طور کامل استفاده کند.




به‌جای تقلید از معماری موتورهای موجود، بهترین راهکارها را از نظر عملکرد، سادگی، قابلیت نگهداری و هزینه منابع بررسی کن.


هر تصمیم معماری باید بر اساس شواهد فنی، Benchmark و تحلیل واقعی اتخاذ شود.




هدف ما صرفاً ساخت یک Game Engine نیست؛ هدف استخراج حداکثر توان سخت‌افزار با کمترین سربار نرم‌افزاری است.




Maximum Realism.
Maximum Performance.
Minimum Overhead.
Native C.
Windows + Linux + Android.
هر ابزار تستی که نیاز بود نصب کن و روی پلتفرم های مختلف تست کن و یک بازی نمونه 3D بسیار بسیار زیبا و خوشگل و واقع گرایانه بساز باهاش. یک بازی 3D رانندگی.

