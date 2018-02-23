# AIMACore

|AIMACore|[![Build Status](https://travis-ci.org/sambitdash/AIMACore.jl.svg?branch=master)](https://travis-ci.org/sambitdash/AIMACore.jl)|[![Build status](https://ci.appveyor.com/api/projects/status/lmk4xl3ieqx1pvfb?svg=true)](https://ci.appveyor.com/project/sambitdash/aimacore-jl)|[![codecov.io](http://codecov.io/github/sambitdash/AIMACore.jl/coverage.svg?branch=master)](http://codecov.io/github/sambitdash/AIMACore.jl?branch=master)|
|---|---|---|---|
|**AIMASamples**|[![Build  Status](https://travis-ci.org/sambitdash/aimasamples.jl.svg?branch=master)](https://travis-ci.org/sambitdash/aimasamples.jl)|[![Build status](https://ci.appveyor.com/api/projects/status/vjxa0f5y60xbc79r?svg=true)](https://ci.appveyor.com/project/sambitdash/aimasamples-jl)|[![codecov.io](http://codecov.io/github/sambitdash/aimasamples.jl/coverage.svg?branch=master)](http://codecov.io/github/sambitdash/aimasamples.jl?branch=master)|

This package is developed by the author as a sample program development of
Artificial Intelligence - A Modern Approach 3rd Edition. This is not the
official sample program source for the book. The solution here is just the
author's own interpretation and does not have any dependency or ratification
from any work carried out as part of the official sample code available at:
[aimacode](https://github.com/aimacode). Anyone interested in the official
version may visit the link provided.

The environment is split into two parts. [AIMACore](https://github.com/sambitdash/AIMACore.jl)
and [AIMASamples](https://github.com/sambitdash/AIMASamples.jl). *AIMACore* contains the core
libraries and *AIMASamples* provides examples to test the core library.
Although, the detailed documentation has not been developed, the intent is
to the keep the essence of the algorithm as sacrosanct as possible. Algorithms
are developed as no side effects such that data types can be governed by
underlying applications. Keeping this generalization in mind the typical
data structures are not mapped to a specific implementation but parametrized.
This way a performance sensitive application may use any data structure that
is optimized for the specific application. For someone developing her own
application can use the *AIMACore* library only. There is no reason to add
the additional test libraries.

This sample program library is built as Julia packages which can be loaded into
 the Julia runtime easily. For testing purposes, data files are downloaded from the
 [aimacode/aima-data](https://github.com/aimacode/aima-data) during the build process.
 
 ## Contribution Guidelines
 
 1. Anyone interested in contributing to the library is free to submit PRs. 
 2. However, it's advisable to create an "Issue" and describe what you are working on and 
 how that can benefit the library. Duplication of efforts and surprises are an unnecessary 
 overheads to planning. It's suggested to review issues and collaborate with someone already 
 working on an issue than create a parallel contribution for the same.
 3. Avoid submitting resolution of multiple issues in one PR. Submission of small PRs makes
 it easier for review. 
 4. Make sure to add elaborate test cases in `AIMASamples` for every code change submitted. 
 Untested code is hard to validate for correctness. 
