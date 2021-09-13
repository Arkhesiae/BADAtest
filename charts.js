
let COUNT = 0
let ABSOLUTE_COUNT = 0

// Chart.defaults.global.defaultFontFamily = 'Product Sans';
// const ctx2 = document.getElementById('conflict_number').getContext('2d');





export function createCharts(dataSets){
    let charts = []
    Object.keys(dataSets).forEach((key) => {
        let min
        let max
        let dataParams = [{
            radius:0,
            borderWidth:1,
            data: dataSets[key].data,
        }]
        switch (key) {
            case "altitude":
                min = 0
                console.log(dataSets)
                let chart = new Chart(dataSets[key].graph, {
                    type: 'line',
                    data: {
                        labels: dataSets.time.data,
                        datasets:
                            [{
                                pointHitRadius :100,
                                label:"altitude",
                                borderColor: 'rgb(110,171,229)',
                                radius:0,
                                borderWidth:1,
                                data: dataSets[key].data,
                            }]
                    },
                    options: {
                        animation: false,
                        scales: {
                            y: {
                                font: {
                                    size: 5
                                },
                                grid: {
                                    display: false
                                },
                                ticks: {
                                    suggestedMin: 0,
                                    suggestedMax: max,
                                },
                                suggestedMin: 0,

                            },
                            x: {
                                grid: {
                                    display: false
                                },

                                type: 'linear',
                                grace: '5%',
                                suggestedMin: 0,
                            },
                            // x: {
                            //     // ticks: {
                            //     //     // For a category axis, the val is the index so the lookup via getLabelForValue is needed
                            //     //     callback: function (val, index) {
                            //     //         // Hide the label of every 2nd dataset
                            //     //         // console.log(index % 100 === 0)
                            //     //         return Math.floor(this.getLabelForValue(val)/60)+":"+this.getLabelForValue(val)%60 ;
                            //     //     },
                            //     //     // color: 'red',
                            //     // }
                            // }
                        },
                        legend: {display: false},
                        title: {
                            display: false,
                            text: key
                        },
                        plugins: {
                            tooltip: {
                                enabled: true,
                                backgroundColor: "rgba(0,100,0,0)",
                                callbacks: {
                                    label: function(context) {
                                        var label = context.dataset.label || '';
                                        if (label) {
                                            label += ': ';
                                        }
                                        if (context.parsed.y !== null) {
                                            label += new Intl.NumberFormat('en-US', { style: 'currency', currency: 'USD' }).format(context.parsed.y);
                                        }
                                        return '';
                                    },
                                    title : function (context){
                                        return ''
                                    },
                                    footer : function(tooltipItems)  {
                                        // console.log(tooltipItems)
                                        return tooltipItems[0].raw + " ft";
                                    }
                                },
                            },
                        }
                    }
                });
                charts.push({chart:chart, label:"altitude"})
                break
            case "ROCD":
                min = 0
                let chartrt = new Chart(dataSets[key].graph, {
                    type: 'line',
                    data: {
                        labels: dataSets.time.data,
                        datasets:
                            [{
                                // pointBackgroundColor : 'rgb(248,56,255)',
                                pointHitRadius :100,
                                // borderColor: [
                                //     'rgb(255, 99, 132)',
                                //     'rgb(255, 159, 64)',
                                //     'rgb(255, 205, 86)',
                                //     'rgb(75, 192, 192)',
                                //     'rgb(54, 162, 235)',
                                //     'rgb(153, 102, 255)',
                                //     'rgb(201, 203, 207)'
                                // ],
                                label:"ROCD",
                                borderColor: 'rgb(158,110,229)',
                                radius:0,
                                borderWidth:1,
                                // borderDash: [3, 5],
                                // steppedLine: true,
                                data: dataSets[key].data,
                            }]
                    },
                    options: {
                        animation: false,
                        scales: {
                            y: {
                                font: {
                                    size: 5
                                },
                                grid: {
                                    display: false
                                },
                                ticks: {
                                    suggestedMin: 0,
                                    suggestedMax: max,
                                },
                                suggestedMin: 0,

                            },
                            x: {
                                grid: {
                                    display: false
                                },

                                type: 'linear',
                                grace: '5%',
                                suggestedMin: 0,
                            },
                            // x: {
                            //     // ticks: {
                            //     //     // For a category axis, the val is the index so the lookup via getLabelForValue is needed
                            //     //     callback: function (val, index) {
                            //     //         // Hide the label of every 2nd dataset
                            //     //         // console.log(index % 100 === 0)
                            //     //         return Math.floor(this.getLabelForValue(val)/60)+":"+this.getLabelForValue(val)%60 ;
                            //     //     },
                            //     //     // color: 'red',
                            //     // }
                            // }
                        },
                        legend: {display: false},
                        title: {
                            display: false,
                            text: key
                        },
                        plugins: {
                            legend: {
                                labels: {
                                    // This more specific font property overrides the global property

                                }
                            },
                            tooltip: {
                                enabled: true,
                                // intersect:false,
                                backgroundColor: "rgba(0,100,0,0)",
                                // mode: 'single',
                                callbacks: {
                                    label: function(context) {
                                        var label = context.dataset.label || '';

                                        return '';
                                    },

                                    title : function (context){
                                        return ''
                                    },

                                    footer : function(tooltipItems)  {
                                        // console.log(tooltipItems)
                                        return tooltipItems[0].raw + " ft/min";

                                    }
                                },
                            },


                        }
                    }
                });
                charts.push({chart:chartrt, label:"ROCD"})
                break
            case "speed":
                min = 0
                dataParams = [
                    {
                        label:"TAS",
                        borderColor: 'rgba(255,74,101)',
                        radius:0,
                        borderDash: [5, 5],
                        borderWidth:1,
                        data: dataSets[key].data.TAS,
                    },
                    {
                        label:"CAS",
                        borderColor: 'rgb(241,166,149)',
                        radius:0,
                        borderWidth:1,
                        data: dataSets.speed.data.CAS,
                    }
                    ,
                ]
                let chartSpeed = new Chart(dataSets.speed.graph, {
                    type: 'line',
                    data: {
                        labels: dataSets.time.data,
                        datasets: dataParams
                    },
                    options: {
                        animation: false,
                        legend: {display: false},
                        title: {
                            display: false,
                            text: key
                        },
                        scales: {
                            x: {
                                grid: {
                                    display: false
                                },
                                type: 'linear',
                                grace: '1%',
                                suggestedMin: 0,

                            },
                            y: {
                                grid: {
                                    display: false
                                },
                                suggestedMin: 0,
                                suggestedMax: 500
                            }
                        }
                    }
                });
                charts.push({chart:chartSpeed, label:"speed"})
                break
        }
    })
    let dataParams = [
        {
            label:"mach",
            borderColor: 'rgb(132,227,177)',
            radius:0,
            // borderDash: [5, 5],
            borderWidth:1,
            data: dataSets.mach.data.mach,
        },
        {
            label:"buffet",
            borderColor: 'rgb(116,130,144)',
            radius:0,
            // borderDash: [5, 5],
            borderWidth:1,
            data: dataSets.mach.data.buffet,
        },
        {
            label:"maxMach",
            borderColor: 'rgb(132,106,106)',
            radius:0,
            // borderDash: [5, 5],
            borderWidth:1,
            data: dataSets.mach.data.maxMach,
        },

    ]
    let chartMach = new Chart(dataSets.mach.graph, {
        type: 'line',
        data: {
            labels: dataSets.time.data,
            datasets: dataParams
        },
        options: {
            animation: false,
            legend: {display: false},
            title: {
                display: false,
                text: "Mach"
            },
            scales: {
                x: {
                    grid: {
                        display: false
                    },
                    type: 'linear',
                    grace: '1%',
                    suggestedMin: 0,

                },
                y: {
                    grid: {
                        display: false
                    },
                    suggestedMin: 0,
                    suggestedMax: 1
                }
            }
        }
    });
    charts.push({chart:chartMach, label:"mach"})

    let chartForce = new Chart(dataSets.force.graph, {
        type: 'line',
        data: {
            labels: dataSets.time.data,
            datasets:  [
                {
                    label:"thrust",
                    borderColor: 'rgb(74,174,255)',
                    radius:0,
                    borderDash: [5, 5],
                    borderWidth:1,
                    // steppedLine: true,
                    data: dataSets.force.data.thrust,
                },
                {
                    label:"maxThrust",
                    borderColor: 'rgba(255,74,101)',
                    radius:0,

                    borderWidth:1,
                    // steppedLine: true,
                    data: dataSets.force.data.maxThrust,
                },
                {
                    label:"drag",
                    borderColor: 'rgb(255,180,0)',
                    radius:0,
                    borderRadius:1,
                    borderDash: [5, 5],
                    borderWidth:1,
                    // steppedLine: true,
                    data: dataSets.force.data.drag,
                }
                ,
            ]
        },
        options: {
            animation: false,
            legend: {display: false},
            title: {
                display: false,

            },
            scales: {
                x: {
                    grid: {
                        display: false
                    },
                    type: 'linear',
                    grace: '5%',
                    suggestedMin: 0,
                    suggestedMax: 500
                },
                y: {
                    grid: {
                        display: false
                    },
                    suggestedMin: 0,
                    suggestedMax: 500
                }
                // x: {
                //     // ticks: {
                //     //     // For a category axis, the val is the index so the lookup via getLabelForValue is needed
                //     //     callback: function (val, index) {
                //     //         // Hide the label of every 2nd dataset
                //     //         // console.log(index % 100 === 0)
                //     //         return Math.floor(this.getLabelForValue(val)/60)+":"+this.getLabelForValue(val)%60 ;
                //     //     },
                //     //     // color: 'red',
                //     // }
                // }
            }
        }
    });
    charts.push({chart:chartForce, label:"force"})
    return charts
}



// var chart = new Chart(ctx, {
//     type: 'line',
//     data: {
//         labels: [0],
//         datasets: [{
//             borderColor: 'rgb(255,74,101)',
//             steppedLine: true,
//             data: [0]
//         }]
//     },
//     options: {
//         legend: {display: false},
//         title: {
//             display: true,
//             text: 'SECTOR LOAD'
//         },
//         scales: {
//             yAxes: [{
//                 gridLines: {
//                     display: false
//                 },
//                 ticks: {
//                     suggestedMin: 0,
//                     suggestedMax: 20
//                 }
//             }]
//         }
//     }
// });


function addData(chart, label, data) {
    chart.data.labels.push(label);
    chart.data.datasets.forEach((dataset) => {
        dataset.data.push(data);

    });
    chart.update();
}

function shiftData(chart) {
    chart.data.labels.shift()
    chart.data.datasets.forEach((dataset) => {
        dataset.data.shift()


    });
    chart.update();
}
